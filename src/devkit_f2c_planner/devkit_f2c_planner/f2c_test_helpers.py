"""Shared helpers for the devkit_f2c_planner unit tests.

- ``planner``: f2c_planner imported even where fields2cover is not installed
  (fields2cover is built in docker/Dockerfile and is not pip-installable in CI).
  Only the shapely-based functions work without it; tests that go through
  ``_run_f2c()`` patch ``planner.f2c`` with ``FakeF2C``.
- ``to_ll`` / ``to_xy``: build fixtures in metres and convert them with the
  planner's own projection. The projection itself is tested independently in
  test_projection.py.
- ``FakeF2C``: records calls and returns canned swaths. It mirrors only the
  fields2cover calls made by ``_run_f2c()``, so it tests the planner's own
  logic (ordering, clipping, projection), not fields2cover.
"""
# pylint: disable=invalid-name,protected-access
import importlib
import importlib.util
import io
import sys
import types
import unittest
from unittest import mock

LAT0, LON0 = 55.0, -1.5


def _import_planner() -> types.ModuleType:
    if 'fields2cover' in sys.modules or importlib.util.find_spec('fields2cover') is not None:
        return importlib.import_module('devkit_f2c_planner.f2c_planner')
    sys.modules['fields2cover'] = types.ModuleType('fields2cover')
    try:
        return importlib.import_module('devkit_f2c_planner.f2c_planner')
    finally:
        del sys.modules['fields2cover']


planner = _import_planner()


def to_ll(points_xy: list, lat0: float = LAT0, lon0: float = LON0) -> list:
    """Local xy metres -> lat/lon, anchored at (lat0, lon0) == xy (0, 0)."""
    return [planner._f2c_xy_to_latlon(x, y, lat0, lon0) for x, y in points_xy]


def to_xy(points_ll: list, lat0: float = LAT0, lon0: float = LON0) -> list:
    """lat/lon -> local xy metres, anchored at (lat0, lon0)."""
    return [planner._f2c_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in points_ll]


def silence_planner_log(test: unittest.TestCase) -> None:
    """The planner logs to stderr with print(); keep test output clean."""
    patcher = mock.patch('sys.stderr', new_callable=io.StringIO)
    patcher.start()
    test.addCleanup(patcher.stop)


class _FakePoint:
    def __init__(self, x: float, y: float, _z: float = 0.0):
        self._x, self._y = x, y

    def getX(self) -> float:
        return self._x

    def getY(self) -> float:
        return self._y


class _FakeRing:
    def __init__(self):
        self.points: list = []

    def addPoint(self, point: _FakePoint) -> None:
        self.points.append((point.getX(), point.getY()))

    def closeRing(self) -> None:
        pass


class FakeCell:
    """f2c.Cell stand-in. ``rings[0]`` is the outer boundary, the rest are hole hints."""

    def __init__(self):
        self.rings: list = []

    def addRing(self, ring: _FakeRing) -> None:
        self.rings.append(ring.points)


class _FakeSequence:
    """Shared size()/getGeometry() shape of F2C's path and cell collections."""

    def __init__(self, items: list):
        self._items = items

    def size(self) -> int:
        return len(self._items)

    def getGeometry(self, index: int):
        return self._items[index]


class _FakeSwath:
    def __init__(self, points_xy: list):
        self._path = _FakeSequence([_FakePoint(x, y) for x, y in points_xy])

    def getPath(self) -> _FakeSequence:
        return self._path


class _FakeSwaths:
    def __init__(self, swaths_xy: list):
        self._swaths = [_FakeSwath(pts) for pts in swaths_xy]

    def size(self) -> int:
        return len(self._swaths)

    def at(self, index: int) -> _FakeSwath:
        return self._swaths[index]


class FakeF2C:
    """Canned stand-in for the ``fields2cover`` module, as used by ``_run_f2c()``.

    swaths_xy       swaths (lists of local-xy points) returned by SG_BruteForce
    headland_cells  cells returned by the headland generator (default: one inset cell)
    headland_error  exception raised by the headland generator instead
    """

    Point = _FakePoint
    LinearRing = _FakeRing
    Cell = FakeCell

    def __init__(self, swaths_xy: list, headland_cells: list | None = None,
                 headland_error: Exception | None = None):
        self.swaths_xy = swaths_xy
        self.inset_cell = FakeCell()
        self.headland_cells = [self.inset_cell] if headland_cells is None else headland_cells
        self.headland_error = headland_error
        self.headland_widths: list = []
        self.swath_calls: list = []  # (angle_rad, tool_width, cell)

    def Cells(self, cell: FakeCell) -> FakeCell:
        return cell

    def HG_Const_gen(self) -> '_FakeHeadlandGenerator':
        return _FakeHeadlandGenerator(self)

    def SG_BruteForce(self) -> '_FakeSwathGenerator':
        return _FakeSwathGenerator(self)


class _FakeHeadlandGenerator:
    def __init__(self, fake: FakeF2C):
        self._fake = fake

    def generateHeadlands(self, _cells: FakeCell, width: float) -> _FakeSequence:
        self._fake.headland_widths.append(width)
        if self._fake.headland_error is not None:
            raise self._fake.headland_error
        return _FakeSequence(self._fake.headland_cells)


class _FakeSwathGenerator:
    def __init__(self, fake: FakeF2C):
        self._fake = fake

    def generateSwaths(self, angle_rad: float, tool_width: float, cell: FakeCell) -> _FakeSwaths:
        self._fake.swath_calls.append((angle_rad, tool_width, cell))
        return _FakeSwaths(self._fake.swaths_xy)
