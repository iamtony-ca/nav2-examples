"""Python codec round trips (mirror of the C++ tests) and the global costmap mirror."""
import numpy as np

from nav_data_tools import codecs


def test_scan_codes_special_values():
    r = np.array([np.nan, np.inf, -np.inf, 70.0, 0.0001, 1.2344, 65.532], np.float32)
    c = codecs.ranges_to_codes(r)
    assert list(c[:4]) == [codecs.CODE_NAN, codecs.CODE_POS_INF, codecs.CODE_NEG_INF,
                           codecs.CODE_SATURATED]
    back = codecs.codes_to_ranges(c)
    assert np.isnan(back[0]) and back[1] == np.inf and back[2] == -np.inf
    assert abs(back[5] - 1.2344) <= 0.0005 + 1e-6
    assert abs(back[6] - 65.532) <= 0.0005 + 1e-6


def test_scan_delta_roundtrip():
    rng = np.random.default_rng(0)
    r = rng.uniform(0.1, 30, 7200).astype(np.float32)
    dec = codecs.ScanDecoder()
    prev = None
    for k in range(20):
        r = np.clip(r + rng.normal(0, 0.02, r.size).astype(np.float32), 0.06, 39)
        enc, payload, prev = codecs.encode_scan(r, prev if k % 10 else None)
        out = dec.decode(enc, r.size, payload)
        assert out is not None
        assert np.max(np.abs(out - r)) <= 0.0005 + 1e-6


def test_scan_delta_needs_key():
    r = np.ones(10, np.float32)
    _, _, codes = codecs.encode_scan(r, None)
    enc, payload, _ = codecs.encode_scan(r * 2, codes)
    assert enc == codecs.ENC_DELTA_MM
    assert codecs.ScanDecoder().decode(enc, 10, payload) is None


def test_zigzag_matches_cpp_definition():
    d = np.array([0, -1, 1, -2, 2, 32767, -32768], np.int16)
    z = codecs.zigzag_encode(d)
    assert list(z) == [0, 1, 2, 3, 4, 65534, 65535]
    assert list(codecs.zigzag_decode(z)) == list(d)


def test_grid_xor_shift_roundtrip():
    rng = np.random.default_rng(1)
    world = rng.choice(np.array([0, 128, 254], np.uint8), size=(800, 800), p=[0.9, 0.05, 0.05])
    dec = codecs.GridDecoder()
    prev = None
    cx = cy = 100
    pcx = pcy = None
    for k in range(30):
        cx += int(rng.integers(-2, 5))
        cy += int(rng.integers(-2, 3))
        cur = world[cy:cy + 300, cx:cx + 300].copy().reshape(-1)
        if prev is None or k % 10 == 0:
            enc, payload, sx, sy = codecs.ENC_RAW, cur.tobytes(), 0, 0
        else:
            sx, sy = cx - pcx, cy - pcy
            payload = (cur ^ codecs.align_shift(prev, 300, 300, sx, sy)).tobytes()
            enc = codecs.ENC_XOR_PREV
        out = dec.decode(enc, 300, 300, sx, sy, payload)
        assert out is not None and np.array_equal(out, cur)
        prev, pcx, pcy = cur, cx, cy


def test_cost_translation_like_nav2():
    t = codecs.COST_TO_OCC
    assert t[0] == 0 and t[253] == 99 and t[254] == 100 and t[255] == -1
    assert t[1] == 1 and t[252] == 1 + (97 * 251) // 251


class _Stamp:
    def __init__(self, ns):
        self.sec, self.nanosec = divmod(ns, 1_000_000_000)


class _H:
    def __init__(self, ns):
        self.stamp = _Stamp(ns)
        self.frame_id = 'map'


class _Md:
    def __init__(self, w, h):
        self.size_x, self.size_y, self.resolution, self.origin = w, h, 0.05, None


class _Full:
    def __init__(self, ns, grid):
        self.header = _H(ns)
        self.metadata = _Md(grid.shape[1], grid.shape[0])
        self.data = grid.tobytes()


class _Upd:
    def __init__(self, ns, x, y, patch):
        self.header = _H(ns)
        self.x, self.y = x, y
        self.size_y, self.size_x = patch.shape
        self.data = patch.tobytes()


def test_global_mirror_replays_updates_newer_than_keyframe():
    truth = np.zeros((50, 60), np.uint8)
    m = codecs.GlobalMirror()
    m.apply_full(_Full(1, truth))
    ups = []
    for i in range(2, 30):
        p = np.full((3, 4), i, np.uint8)
        truth[i:i + 3, i:i + 4] = p
        ups.append(_Upd(i, i, i, p))
    # keyframe (get_costmap) taken at stamp 15, arrives after updates up to 29
    snap = truth.copy()
    for u in ups:
        m.apply_update(u)
    key = np.zeros_like(truth)
    for i in range(2, 16):
        key[i:i + 3, i:i + 4] = i
    m2 = codecs.GlobalMirror()
    for u in ups:                       # updates first (recorded before the key response)
        m2.apply_update(u)
    m2.apply_full(_Full(15, key))
    assert np.array_equal(m.grid, snap)
    assert np.array_equal(m2.grid, snap)


def test_global_mirror_drops_out_of_bounds():
    m = codecs.GlobalMirror()
    m.apply_full(_Full(1, np.zeros((10, 10), np.uint8)))
    m.apply_update(_Upd(2, 8, 8, np.ones((4, 4), np.uint8)))
    assert m.dropped == 1 and m.grid.sum() == 0
