#!/usr/bin/env python3
"""Latch-and-republish node for driving Nav2 static_layer resolution at runtime.

Subscribes to a source OccupancyGrid (e.g. ``/map`` from ``map_server``), keeps
the latest message, and republishes it to an output topic that the costmap's
``static_layer`` subscribes to. On a trigger it can resample the grid to a new
resolution so that ``StaticLayer::processMap()`` resizes the master costmap and,
through ``LayeredCostmap::resizeMap() -> matchSize()``, every ``CostmapLayer``
including the ``obstacle_layer``.

Why resampling is required:
    With ``static_layer`` present the global costmap's size is locked. The dynamic
    ``resolution`` parameter is ignored. The only runtime path that re-resizes the
    master grid is ``processMap()``, and its resize condition only fires when the
    incoming map differs in size/resolution/origin. Republishing the map *as-is*
    therefore changes nothing; the republished map must carry a different
    ``info.resolution`` (with the cell data resampled to match) to take effect.

Target: ROS 2 Jazzy / Nav2 Jazzy (rclpy, Python 3.12, PEP 8).
"""

from __future__ import annotations

import copy

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float64


def latched_map_qos() -> QoSProfile:
    """QoS matching map_server and Nav2 static_layer (transient_local, reliable).

    static_layer (jazzy) subscribes with transient_local + reliable + keep_last(1)
    when ``map_subscribe_transient_local`` is true (its default). The publisher
    must match so the latched message is delivered on late subscription.
    """
    return QoSProfile(
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class MapRepublisher(Node):
    """Hold the latest map and republish it (optionally resampled) on trigger."""

    def __init__(self) -> None:
        super().__init__("map_republisher")

        self.declare_parameter("source_topic", "/map")
        self.declare_parameter("output_topic", "/map_resized")
        self.declare_parameter("trigger_topic", "/republish_map")
        # Republish (at the current target resolution) whenever a new source map
        # arrives, so the costmap always has a map even before the first trigger.
        self.declare_parameter("auto_publish_on_receive", True)

        self._source_topic = self._str_param("source_topic")
        self._output_topic = self._str_param("output_topic")
        trigger_topic = self._str_param("trigger_topic")
        self._auto_publish = (
            self.get_parameter("auto_publish_on_receive")
            .get_parameter_value()
            .bool_value
        )

        self._latest_map: OccupancyGrid | None = None
        self._target_res: float = 0.0  # <= 0 means "republish as-is"

        latched = latched_map_qos()
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._source_topic, self._on_map, latched
        )
        self._map_pub = self.create_publisher(
            OccupancyGrid, self._output_topic, latched
        )
        self._trigger_sub = self.create_subscription(
            Float64, trigger_topic, self._on_trigger, 10
        )

        self.get_logger().info(
            "map_republisher ready | sub=%s pub=%s trigger=%s "
            "(Float64 = target resolution [m/cell], <=0 = as-is)"
            % (self._source_topic, self._output_topic, trigger_topic)
        )

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self.get_logger().info(
            "Stored source map: %dx%d @ %.4f m/cell"
            % (msg.info.width, msg.info.height, msg.info.resolution)
        )
        if self._auto_publish:
            self._publish_current()

    def _on_trigger(self, msg: Float64) -> None:
        if self._latest_map is None:
            self.get_logger().warn("Trigger received but no map stored yet; ignoring.")
            return
        self._target_res = float(msg.data)
        self._publish_current()

    def _publish_current(self) -> None:
        assert self._latest_map is not None
        if self._target_res <= 0.0:
            out = copy.deepcopy(self._latest_map)  # as-is
            self.get_logger().info("Publishing stored map as-is.")
        else:
            out = self._resample(self._latest_map, self._target_res)
            self.get_logger().info(
                "Publishing resampled map @ %.4f m/cell (%dx%d)."
                % (out.info.resolution, out.info.width, out.info.height)
            )
        out.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(out)

    @staticmethod
    def _resample(src: OccupancyGrid, target_res: float) -> OccupancyGrid:
        """Nearest-neighbor resample to a new resolution.

        The physical extent and the origin are preserved; only the cell size
        (and therefore width/height) changes. Occupancy semantics (-1 unknown,
        0..100) are kept by nearest sampling rather than interpolation.
        """
        src_res = src.info.resolution
        src_w, src_h = src.info.width, src.info.height

        # Row-major OccupancyGrid -> 2D array indexed [row(y), col(x)].
        src_grid = np.asarray(src.data, dtype=np.int16).reshape(src_h, src_w)

        # New cell counts that preserve the physical size in meters.
        dst_w = max(1, int(round(src_w * src_res / target_res)))
        dst_h = max(1, int(round(src_h * src_res / target_res)))

        # Map each destination cell center (meters from origin) to nearest source cell.
        dst_x_m = (np.arange(dst_w) + 0.5) * target_res
        dst_y_m = (np.arange(dst_h) + 0.5) * target_res
        src_col = np.clip((dst_x_m / src_res).astype(int), 0, src_w - 1)
        src_row = np.clip((dst_y_m / src_res).astype(int), 0, src_h - 1)

        dst_grid = src_grid[np.ix_(src_row, src_col)]

        out = OccupancyGrid()
        out.header = copy.deepcopy(src.header)
        out.info = copy.deepcopy(src.info)  # deep copy so we never mutate the stored map
        out.info.resolution = float(target_res)
        out.info.width = dst_w
        out.info.height = dst_h
        # out.info.origin intentionally left unchanged.
        out.data = dst_grid.astype(np.int8).reshape(-1).tolist()
        return out


def main() -> None:
    rclpy.init()
    node = MapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
