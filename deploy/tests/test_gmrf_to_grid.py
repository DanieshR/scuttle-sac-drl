from gmrf_to_grid import parse_mean_csv, mean_to_int8, flatten_occupancy


def test_parse_mean_csv_basic():
    rows = parse_mean_csv("0.0,0.5\n1.0,\n")
    assert rows == [[0.0, 0.5], [1.0, None]]


def test_parse_mean_csv_handles_nan_and_blanks():
    rows = parse_mean_csv("nan, ,0.25")
    assert rows == [[None, None, 0.25]]


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
