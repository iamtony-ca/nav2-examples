"""Python side of the CompactScan / CompactGrid codecs and the global costmap mirror.

Must stay bit-compatible with nav_data_recorder/src/{scan,grid}_codec.cpp.
The encoders here are used by tests and tools that synthesize data.
"""
from collections import deque

import numpy as np

ENC_ABS_MM = 0
ENC_DELTA_MM = 1
ENC_RAW = 0
ENC_XOR_PREV = 1

CODE_NAN = 0
CODE_NEG_INF = 65533
CODE_POS_INF = 65534
CODE_SATURATED = 65535


# ---------------------------------------------------------------- scan
def ranges_to_codes(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=np.float32)
    mm = np.round(r.astype(np.float64) * 1000.0)
    codes = np.clip(mm, 1, 65532)
    codes = np.where(mm >= 65533.0, CODE_SATURATED, codes)
    codes = np.where(np.isposinf(r), CODE_POS_INF, codes)
    codes = np.where(np.isneginf(r), CODE_NEG_INF, codes)
    codes = np.where(np.isnan(r), CODE_NAN, codes)
    return codes.astype(np.uint16)


def codes_to_ranges(c: np.ndarray) -> np.ndarray:
    c = np.asarray(c, dtype=np.uint16)
    r = c.astype(np.float32) / np.float32(1000.0)
    r[c == CODE_NAN] = np.nan
    r[c == CODE_NEG_INF] = -np.inf
    r[c == CODE_POS_INF] = np.inf
    r[c == CODE_SATURATED] = np.float32(65.535)
    return r


def zigzag_encode(d: np.ndarray) -> np.ndarray:
    d = d.astype(np.int16)
    return ((d.astype(np.uint16) << 1) ^ (d >> 15).astype(np.uint16)).astype(np.uint16)


def zigzag_decode(z: np.ndarray) -> np.ndarray:
    z = z.astype(np.uint16)
    return ((z >> 1) ^ (-(z & 1).astype(np.int32)).astype(np.uint16)).astype(np.int16)


class ScanDecoder:
    """Stateful per source topic. decode() returns float32 ranges or None (no keyframe yet)."""

    def __init__(self):
        self.prev = None

    def decode(self, encoding: int, beam_count: int, payload) -> 'np.ndarray | None':
        raw = np.frombuffer(bytes(payload), dtype='<u2')
        if raw.size != beam_count:
            return None
        if encoding == ENC_ABS_MM:
            codes = raw.astype(np.uint16)
        elif encoding == ENC_DELTA_MM:
            if self.prev is None or self.prev.size != beam_count:
                return None
            codes = (self.prev + zigzag_decode(raw).astype(np.uint16)).astype(np.uint16)
        else:
            return None
        self.prev = codes
        return codes_to_ranges(codes)


def encode_scan(ranges, prev_codes, decimation=1):
    """(encoding, payload bytes, codes) - reference encoder for tests."""
    codes = ranges_to_codes(np.asarray(ranges, dtype=np.float32)[::decimation])
    if prev_codes is None or prev_codes.size != codes.size:
        return ENC_ABS_MM, codes.astype('<u2').tobytes(), codes
    d = (codes.astype(np.int32) - prev_codes.astype(np.int32)).astype(np.int16)
    return ENC_DELTA_MM, zigzag_encode(d).astype('<u2').tobytes(), codes


# ---------------------------------------------------------------- grid
def align_shift(prev: np.ndarray, w: int, h: int, sx: int, sy: int) -> np.ndarray:
    """aligned[y][x] = prev[y+sy][x+sx] inside, else 0. prev is flat row-major w*h."""
    out = np.zeros((h, w), dtype=np.uint8)
    p = prev.reshape(h, w)
    x0, x1 = max(0, -sx), min(w, w - sx)
    y0, y1 = max(0, -sy), min(h, h - sy)
    if x1 > x0 and y1 > y0:
        out[y0:y1, x0:x1] = p[y0 + sy:y1 + sy, x0 + sx:x1 + sx]
    return out.reshape(-1)


class GridDecoder:
    def __init__(self):
        self.prev = None
        self.size = (0, 0)

    def decode(self, encoding, size_x, size_y, shift_x, shift_y, payload) -> 'np.ndarray | None':
        cur = np.frombuffer(bytes(payload), dtype=np.uint8)
        if cur.size != size_x * size_y:
            return None
        if encoding == ENC_RAW:
            out = cur.copy()
        elif encoding == ENC_XOR_PREV:
            if self.prev is None or self.size != (size_x, size_y):
                return None
            out = cur ^ align_shift(self.prev, size_x, size_y, shift_x, shift_y)
        else:
            return None
        self.prev = out
        self.size = (size_x, size_y)
        return out


# nav2 costmap_2d_publisher cost translation (0..255 -> OccupancyGrid -1..100)
COST_TO_OCC = np.array(
    [0] + [1 + (97 * (i - 1)) // 251 for i in range(1, 253)] + [99, 100, -1], dtype=np.int8)


def costmap_to_occupancy(data: np.ndarray) -> np.ndarray:
    return COST_TO_OCC[np.asarray(data, dtype=np.uint8)]


# ---------------------------------------------------------------- global costmap mirror
def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class GlobalMirror:
    """Full grid (topic FULL or get_costmap keyframe) + CostmapUpdate rectangles.

    A full replaces the grid unless it is older than the
    current one; updates stamped after the full are applied (also those that arrived before the
    full, kept in a small replay buffer); updates outside the grid are dropped.
    """

    def __init__(self, replay_max=256):
        self.grid = None           # np.uint8 (h, w)
        self.meta = None           # (resolution, size_x, size_y, origin pose msg, frame_id)
        self.full_stamp = -1
        self.replay = deque(maxlen=replay_max)
        self.dropped = 0
        self.dirty = False

    def apply_full(self, costmap_msg) -> bool:
        md = costmap_msg.metadata
        n = md.size_x * md.size_y
        if md.size_x == 0 or md.size_y == 0 or len(costmap_msg.data) != n:
            return False
        st = _stamp_ns(costmap_msg.header.stamp)
        same_geom = self.meta is not None and self.meta[1:3] == (md.size_x, md.size_y)
        if same_geom and st < self.full_stamp:
            return False
        self.grid = np.frombuffer(bytes(costmap_msg.data), dtype=np.uint8).reshape(
            md.size_y, md.size_x).copy()
        self.meta = (md.resolution, md.size_x, md.size_y, md.origin, costmap_msg.header.frame_id)
        self.full_stamp = st
        for u in list(self.replay):
            if _stamp_ns(u.header.stamp) > st:
                self._paint(u)
        self.dirty = True
        return True

    def apply_update(self, upd):
        self.replay.append(upd)
        if self.grid is None or _stamp_ns(upd.header.stamp) <= self.full_stamp:
            return
        self._paint(upd)

    def _paint(self, u):
        h, w = self.grid.shape
        if u.x + u.size_x > w or u.y + u.size_y > h or len(u.data) != u.size_x * u.size_y:
            self.dropped += 1
            return
        self.grid[u.y:u.y + u.size_y, u.x:u.x + u.size_x] = np.frombuffer(
            bytes(u.data), dtype=np.uint8).reshape(u.size_y, u.size_x)
        self.dirty = True
