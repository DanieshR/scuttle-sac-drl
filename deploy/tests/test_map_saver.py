from datetime import datetime

from map_saver import resolve_save_dir, grid_to_csv_rows


def test_resolve_save_dir_with_base():
    d = resolve_save_dir('/tmp/maps', datetime(2026, 6, 24, 14, 5, 9))
    assert d == '/tmp/maps/20260624-140509'


def test_resolve_save_dir_default(monkeypatch):
    monkeypatch.setenv('HOME', '/home/tester')
    d = resolve_save_dir(None, datetime(2026, 6, 24, 14, 5, 9))
    assert d == '/home/tester/scuttle_maps/20260624-140509'


def test_resolve_save_dir_empty_string_is_default(monkeypatch):
    monkeypatch.setenv('HOME', '/home/tester')
    d = resolve_save_dir('', datetime(2026, 1, 2, 3, 4, 5))
    assert d == '/home/tester/scuttle_maps/20260102-030405'


def test_grid_to_csv_rows_centers_cells():
    # 2x1 grid, 0.5 m cells, origin (1.0, 2.0), values [40, 60]
    rows = grid_to_csv_rows(2, 1, 0.5, 1.0, 2.0, [40, 60])
    assert rows == [
        (1.25, 2.25, 40),
        (1.75, 2.25, 60),
    ]
