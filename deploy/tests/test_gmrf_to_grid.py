from gmrf_to_grid import parse_mean_csv, mean_to_int8, flatten_occupancy, snap_origin


def test_parse_mean_csv_basic():
    rows = parse_mean_csv("0.0,0.5\n1.0,\n")
    assert rows == [[0.0, 0.5], [1.0, None]]


def test_parse_mean_csv_handles_nan_and_blanks():
    rows = parse_mean_csv("nan, ,0.25")
    assert rows == [[None, None, 0.25]]


def test_snap_origin_matches_gmrf_anchor():
    # GMRF anchors at cell_size*round(min/cell_size) (gmrf_map.cpp).
    assert snap_origin(0.0, 0.5) == 0.0
    assert snap_origin(0.24, 0.5) == 0.0      # rounds down to nearest 0.5
    assert snap_origin(0.26, 0.5) == 0.5      # rounds up to nearest 0.5
    assert snap_origin(-1.3, 0.5) == -1.5     # negative origin snaps too
    assert snap_origin(2.5, 0.5) == 2.5       # already aligned stays put
    # half-away-from-zero like C++ std::round, not Python banker's rounding:
    assert snap_origin(0.25, 0.5) == 0.5      # 0.5 boundary rounds up (not 0.0)
    assert snap_origin(-0.25, 0.5) == -0.5    # symmetric for negatives


def test_mean_to_int8_scales_and_clamps():
    assert mean_to_int8(None) == -1
    assert mean_to_int8(0.0) == 0
    assert mean_to_int8(0.5) == 50
    assert mean_to_int8(1.0) == 100
    assert mean_to_int8(1.7) == 100      # clamp high
    assert mean_to_int8(-0.3) == 0       # clamp low


def test_flatten_occupancy_dims_and_flip():
    rows = [[0.0, 0.0], [1.0, 1.0]]      # row 0 = top in CSV
    data, w, h = flatten_occupancy(rows, flip_vertical=True)
    assert (w, h) == (2, 2)
    # flipped: bottom row (originally row 1) comes first in OccupancyGrid.data
    assert data == [100, 100, 0, 0]


def test_flatten_occupancy_ragged_rows_padded_unknown():
    rows = [[0.5], [0.5, 0.5]]
    data, w, h = flatten_occupancy(rows, flip_vertical=False)
    assert (w, h) == (2, 2)
    assert data == [50, -1, 50, 50]      # first row padded with unknown
