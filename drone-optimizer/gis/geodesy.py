"""
geodesy.py
==========
Real-world distance and projection primitives. No flat-earth approximations:
distances use the WGS84 ellipsoid (pyproj.Geod), and planar work is done in the
correct local UTM zone so that 1 unit == 1 metre with minimal distortion.

This module is the reason the routing graph reflects real geography rather than
an abstract square grid.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pyproj import Geod, Transformer, CRS

# WGS84 ellipsoid — the datum of GPS, SRTM, Copernicus DEM, and OSM.
_GEOD = Geod(ellps="WGS84")


def geodesic_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """Ellipsoidal (geodesic) distance in metres between two lon/lat points.

    This is the true surface distance on WGS84 — correct to millimetres — not a
    haversine sphere approximation. Used for every horizontal edge length.
    """
    _, _, dist = _GEOD.inv(lon1, lat1, lon2, lat2)
    return dist


def utm_epsg_for(lon: float, lat: float) -> int:
    """EPSG code of the UTM zone containing a lon/lat point.

    UTM gives a metric, conformal local projection. Picking the right zone keeps
    distance/area distortion below ~0.1% across a sector, so planar algorithms
    (A* heuristics, polygon tests) stay accurate.
    """
    zone = int((lon + 180.0) / 6.0) + 1
    return (32600 if lat >= 0 else 32700) + zone


@dataclass
class LocalProjector:
    """Projects lon/lat <-> local UTM metres for a study area.

    Built once from a representative point (e.g. the sector centroid); all
    features in the sector share the same zone so their metres are comparable.
    """
    epsg: int

    @classmethod
    def for_point(cls, lon: float, lat: float) -> "LocalProjector":
        return cls(epsg=utm_epsg_for(lon, lat))

    def __post_init__(self):
        self._fwd = Transformer.from_crs("EPSG:4326", f"EPSG:{self.epsg}",
                                         always_xy=True)
        self._inv = Transformer.from_crs(f"EPSG:{self.epsg}", "EPSG:4326",
                                          always_xy=True)

    def to_metres(self, lon: float, lat: float) -> tuple[float, float]:
        x, y = self._fwd.transform(lon, lat)
        return x, y

    def to_lonlat(self, x: float, y: float) -> tuple[float, float]:
        lon, lat = self._inv.transform(x, y)
        return lon, lat

    @property
    def crs(self) -> CRS:
        return CRS.from_epsg(self.epsg)


def slant_distance_m(horizontal_m: float, elev_from_m: float,
                     elev_to_m: float) -> float:
    """True 3-D straight-line distance given horizontal distance + endpoints.

    A 5 km hop that also climbs 2 km is longer than 5 km; this is the Pythagorean
    combination used wherever the physical path length matters (e.g. radio
    line-of-sight range checks).
    """
    dz = elev_to_m - elev_from_m
    return math.hypot(horizontal_m, dz)


def bearing_deg(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """Initial forward azimuth (degrees, 0=N, clockwise) from point 1 to 2."""
    az, _, _ = _GEOD.inv(lon1, lat1, lon2, lat2)
    return az % 360.0
