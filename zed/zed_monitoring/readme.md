# ZED Box to LaserScan Converter

A ROS 2 (Jazzy) Python package that precisely projects ZED 3D Object Detections (`zed_interfaces/msg/ObjectsStamped`) onto a 2D plane as a `sensor_msgs/msg/LaserScan`. This is specifically designed to inject dynamic 3D obstacles directly into Nav2's 2D Local Costmap.

## Features

- **Z-Axis Filtering**: Ignores objects above the robot's collision height or ground noise.
- **Raycasting Projection**: Approximates 3D bounding boxes into accurate 2D footprints with hidden surface removal.
- **Ghost/Noise Filtering**: Rejects objects based on high velocity at edge distances or `SEARCHING` tracking states.
- **Nav2 Compliant**: Handles `inf` values using C-style array structures to bypass strict Python constraints, allowing perfect free-space raytracing in Nav2 ObstacleLayers.

## Requirements

- ROS 2 Jazzy
- `sensor_msgs`
- `zed_interfaces`

## Nodes

### `zed_monitoring_node`

#### Subscribed Topics

- `/zed/zed_node/obj_det/objects` (`zed_interfaces/msg/ObjectsStamped`): 3D bounding box data from the ZED SDK.

#### Published Topics

- `/zed_object_scan` (`sensor_msgs/msg/LaserScan`): The projected 2D footprint scan.

#### Parameters

All parameters can be configured in `config/params.yaml`.

| Parameter               | Type   | Default                         | Description                                                |
| ----------------------- | ------ | ------------------------------- | ---------------------------------------------------------- |
| `objects_topic`         | string | `/zed/zed_node/obj_det/objects` | Input topic name                                           |
| `scan_topic`            | string | `/zed_object_scan`              | Output topic name                                          |
| `min_z_height`          | float  | `0.1`                           | Minimum Z height (m) to consider an object                 |
| `max_z_height`          | float  | `2.0`                           | Maximum Z height (m) to consider an object                 |
| `angle_min`             | float  | `-pi`                           | LaserScan start angle                                      |
| `angle_max`             | float  | `pi`                            | LaserScan end angle                                        |
| `angle_increment`       | float  | `0.0087`                        | Angular resolution (default 0.5 deg)                       |
| `range_min`             | float  | `0.2`                           | Minimum scan range (m)                                     |
| `range_max`             | float  | `20.0`                          | Maximum scan range (m)                                     |
| `projection_resolution` | float  | `0.02`                          | Distance between points during line segment projection (m) |

## Build & Run

**1. Build the package (e.g., inside your Docker container):**

```bash
colcon build --packages-select zed_monitoring --symlink-install
source install/setup.bash
```
