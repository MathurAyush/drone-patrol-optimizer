"""
airspace.py
===========
Restricted-airspace handling. No-fly zones are real polygons (loaded from
GeoJSON/shapefile via geopandas) in geographic coordinates. A candidate flight
edge is rejected if its great-circle segment enters any restricted polygon.

Uses shapely's robust geometry predicates — the same engine behind PostGIS — so
containment and intersection are exact, not bounding-box guesses.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import geopandas as gpd
from shapely.geometry import LineString, Point
from shapely.ops import unary_union


@dataclass
class Airspace:
    """A set of restricted polygons with fast crossing tests."""
    path: str | None = None
    _zones: list = field(default_factory=list)

    def __post_init__(self):
        if self.path:
            gdf = gpd.read_file(self.path)
            # Work in WGS84 lon/lat to match feature coordinates.
            if gdf.crs and gdf.crs.to_epsg() != 4326:
                gdf = gdf.to_crs(epsg=4326)
            self._names = list(gdf.get("name", [f"NFZ{i}" for i in range(len(gdf))]))
            self._zones = list(gdf.geometry)
            self._union = unary_union(self._zones) if self._zones else None
        else:
            self._names, self._union = [], None

    @property
    def count(self) -> int:
        return len(self._zones)

    def contains_point(self, lon: float, lat: float) -> bool:
        """Is this point inside any restricted polygon?"""
        if self._union is None:
            return False
        return self._union.contains(Point(lon, lat))

    def crosses(self, lon1: float, lat1: float,
                lon2: float, lat2: float) -> bool:
        """Does the segment between two points enter restricted airspace?

        Catches both endpoints-inside and fly-through cases (a straight leg that
        clips a corner of a no-fly polygon).
        """
        if self._union is None:
            return False
        seg = LineString([(lon1, lat1), (lon2, lat2)])
        return seg.intersects(self._union)

    def blocking_zone(self, lon1: float, lat1: float,
                      lon2: float, lat2: float) -> str | None:
        """Name of the first restricted zone a segment hits, or None."""
        seg = LineString([(lon1, lat1), (lon2, lat2)])
        for name, poly in zip(self._names, self._zones):
            if seg.intersects(poly):
                return name
        return None
