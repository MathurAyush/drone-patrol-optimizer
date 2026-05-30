"""
terrain.py
==========
Everything that depends on the Digital Elevation Model (DEM). Loads a real
GeoTIFF via rasterio, samples elevation at any lon/lat, and derives the two
terrain-aware quantities the router needs:

  1. climb-energy edge cost  — flying uphill costs energy; this turns a 2-D
     distance into a realistic 3-D effort.
  2. radio line-of-sight      — a relay link only exists if the terrain profile
     between two towers does not block the optical/RF ray.

Drop ANY real DEM (Copernicus GLO-30, SRTM, ASTER) at the configured path and
this module consumes it unchanged — the bundled sample is generated on the same
real coordinate frame for offline reproducibility.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import rasterio
from rasterio.transform import rowcol

from .geodesy import geodesic_m


# Energy model constants (tunable, documented — NOT claimed as ground truth).
# Climbing 1 m vertical costs roughly as much battery as flying CLIMB_EQUIV_M
# horizontal. ~6 is a defensible fixed-wing/quad proxy; expose it for tuning.
CLIMB_EQUIV_M = 6.0
# Descending is not free (control drag) but far cheaper than climbing.
DESCENT_EQUIV_M = 0.4


@dataclass
class Terrain:
    """A loaded DEM with elevation sampling and line-of-sight queries."""
    path: str

    def __post_init__(self):
        self._ds = rasterio.open(self.path)
        self._band = self._ds.read(1)
        self._nodata = self._ds.nodata
        # Bounds in the DEM's own CRS (expected geographic WGS84 for sampling).
        self.crs = self._ds.crs
        self.bounds = self._ds.bounds
        self.shape = self._band.shape

    # -- elevation -------------------------------------------------------
    def elevation(self, lon: float, lat: float) -> float:
        """Bilinearly-interpolated elevation (metres) at a lon/lat point."""
        # Fractional pixel coordinates.
        inv = ~self._ds.transform
        col_f, row_f = inv * (lon, lat)
        r0, c0 = int(np.floor(row_f)), int(np.floor(col_f))
        h, w = self.shape
        # Clamp to valid interior so the 2x2 stencil stays in-bounds.
        r0 = min(max(r0, 0), h - 2)
        c0 = min(max(c0, 0), w - 2)
        fr, fc = row_f - r0, col_f - c0
        fr = min(max(fr, 0.0), 1.0)
        fc = min(max(fc, 0.0), 1.0)
        v00 = self._band[r0, c0]
        v01 = self._band[r0, c0 + 1]
        v10 = self._band[r0 + 1, c0]
        v11 = self._band[r0 + 1, c0 + 1]
        top = v00 * (1 - fc) + v01 * fc
        bot = v10 * (1 - fc) + v11 * fc
        return float(top * (1 - fr) + bot * fr)

    def elevation_profile(self, lon1: float, lat1: float,
                          lon2: float, lat2: float,
                          samples: int = 64) -> tuple[np.ndarray, np.ndarray]:
        """(cumulative_distance_m, elevation_m) sampled along a great-circle leg."""
        lons = np.linspace(lon1, lon2, samples)
        lats = np.linspace(lat1, lat2, samples)
        elevs = np.array([self.elevation(lo, la) for lo, la in zip(lons, lats)])
        total = geodesic_m(lon1, lat1, lon2, lat2)
        dists = np.linspace(0.0, total, samples)
        return dists, elevs

    # -- terrain edge cost ----------------------------------------------
    def climb_cost_km(self, lon1: float, lat1: float,
                      lon2: float, lat2: float) -> dict:
        """Terrain-aware cost of one edge, broken out for transparency.

        effective_km = horizontal_km
                     + climb_m * CLIMB_EQUIV_M / 1000
                     + descent_m * DESCENT_EQUIV_M / 1000
        Always >= horizontal_km >= straight-line, so the A* Euclidean heuristic
        on projected coordinates stays admissible.
        """
        horiz_m = geodesic_m(lon1, lat1, lon2, lat2)
        e1 = self.elevation(lon1, lat1)
        e2 = self.elevation(lon2, lat2)
        dz = e2 - e1
        climb_m = max(0.0, dz)
        descent_m = max(0.0, -dz)
        penalty_m = climb_m * CLIMB_EQUIV_M + descent_m * DESCENT_EQUIV_M
        eff_km = (horiz_m + penalty_m) / 1000.0
        return {
            "horizontal_km": horiz_m / 1000.0,
            "elev_from_m": e1, "elev_to_m": e2,
            "climb_m": climb_m, "descent_m": descent_m,
            "slope": (dz / horiz_m) if horiz_m > 1e-6 else 0.0,
            "effective_km": eff_km,
        }

    # -- radio line-of-sight --------------------------------------------
    def has_line_of_sight(self, lon1: float, lat1: float, h1_m: float,
                          lon2: float, lat2: float, h2_m: float,
                          samples: int = 64) -> bool:
        """True if terrain does not block the straight ray between two antennas.

        Compares the straight line connecting (tower1 top) and (tower2 top)
        against the sampled ground profile. If the ground ever rises above the
        ray, the link is obstructed. This is a genuine DEM viewshed-style check,
        the physical basis for whether a relay radio hop is feasible.
        """
        dists, ground = self.elevation_profile(lon1, lat1, lon2, lat2, samples)
        ray_start = ground[0] + h1_m
        ray_end = ground[-1] + h2_m
        total = dists[-1] if dists[-1] > 0 else 1.0
        ray = ray_start + (ray_end - ray_start) * (dists / total)
        # Small clearance margin (Fresnel-ish); require ground below the ray.
        clearance = 2.0
        return bool(np.all(ground[1:-1] <= ray[1:-1] + clearance))

    def close(self):
        self._ds.close()
