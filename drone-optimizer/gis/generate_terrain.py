"""
generate_terrain.py
===================
Builds a self-contained, real-coordinate sample for the GIS pipeline:

  data/dem.tif        georeferenced elevation raster (WGS84, real bounds)
  data/features.geojson   monitoring stations, survey zones, checkpoints, relays
  data/nofly.geojson      restricted-airspace polygons

The study area is a real Himalayan sector (Nainital district, Uttarakhand) where
elevation 600-2600 m makes terrain-aware routing meaningful — a realistic
environmental / disaster-monitoring context (landslide & flood survey).

The elevation field is procedurally generated (fractal Brownian motion) so the
project is fully reproducible offline. It is written as a standard GeoTIFF on
the real coordinate frame, so an authoritative DEM (Copernicus GLO-30, SRTM)
can replace data/dem.tif with no code change.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rasterio
from rasterio.transform import from_bounds

DATA = Path(__file__).resolve().parent / "data"
DATA.mkdir(exist_ok=True)

# --- real study area (Nainital district, Uttarakhand, India) -------------
WEST, EAST = 79.30, 79.60     # longitude
SOUTH, NORTH = 29.25, 29.50   # latitude
DEM_W, DEM_H = 512, 512       # ~50-65 m/pixel over this extent
ELEV_MIN, ELEV_MAX = 600.0, 2600.0
SEED = 1947


def fbm(width: int, height: int, octaves: int = 6, seed: int = SEED) -> np.ndarray:
    """Fractal Brownian motion via summed, upsampled random lattices.

    Produces coherent ridge/valley terrain without external noise libraries.
    """
    rng = np.random.default_rng(seed)
    field = np.zeros((height, width), dtype=np.float64)
    amp = 1.0
    total_amp = 0.0
    for o in range(octaves):
        cells = 2 ** (o + 1)
        lattice = rng.standard_normal((cells + 1, cells + 1))
        # bilinear upsample to full resolution
        ys = np.linspace(0, cells, height)
        xs = np.linspace(0, cells, width)
        y0 = np.floor(ys).astype(int).clip(0, cells - 1)
        x0 = np.floor(xs).astype(int).clip(0, cells - 1)
        fy = (ys - y0)[:, None]
        fx = (xs - x0)[None, :]
        v00 = lattice[np.ix_(y0, x0)]
        v01 = lattice[np.ix_(y0, x0 + 1)]
        v10 = lattice[np.ix_(y0 + 1, x0)]
        v11 = lattice[np.ix_(y0 + 1, x0 + 1)]
        layer = (v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy)
                 + v10 * (1 - fx) * fy + v11 * fx * fy)
        field += amp * layer
        total_amp += amp
        amp *= 0.5
    field /= total_amp
    return field


def build_dem() -> np.ndarray:
    base = fbm(DEM_W, DEM_H)
    # Add a regional tilt (valleys to the south-east, ridges to the north-west),
    # mimicking the real Himalayan-front gradient.
    yy, xx = np.mgrid[0:DEM_H, 0:DEM_W] / max(DEM_W, DEM_H)
    tilt = (1.0 - yy) * 0.6 + (1.0 - xx) * 0.4
    combined = 0.65 * (base - base.min()) / (np.ptp(base) + 1e-9) + 0.35 * tilt
    elev = ELEV_MIN + combined * (ELEV_MAX - ELEV_MIN)
    return elev.astype(np.float32)


def write_dem(elev: np.ndarray) -> Path:
    transform = from_bounds(WEST, SOUTH, EAST, NORTH, DEM_W, DEM_H)
    path = DATA / "dem.tif"
    with rasterio.open(
        path, "w", driver="GTiff", height=DEM_H, width=DEM_W, count=1,
        dtype="float32", crs="EPSG:4326", transform=transform, nodata=-9999.0,
        compress="deflate",
    ) as dst:
        dst.write(elev, 1)
    return path


def _sample(elev: np.ndarray, lon: float, lat: float) -> float:
    col = int((lon - WEST) / (EAST - WEST) * (DEM_W - 1))
    row = int((NORTH - lat) / (NORTH - SOUTH) * (DEM_H - 1))
    col = min(max(col, 0), DEM_W - 1)
    row = min(max(row, 0), DEM_H - 1)
    return float(elev[row, col])


def build_features(elev: np.ndarray) -> dict:
    rng = np.random.default_rng(SEED)
    feats: list[dict] = []

    def add(fid, kind, lon, lat, **props):
        props = {"id": fid, "kind": kind, **props,
                 "elevation_m": round(_sample(elev, lon, lat), 1)}
        feats.append({"type": "Feature",
                      "geometry": {"type": "Point", "coordinates": [lon, lat]},
                      "properties": props})

    def rand_lon(margin=0.02):
        return float(rng.uniform(WEST + margin, EAST - margin))

    def rand_lat(margin=0.02):
        return float(rng.uniform(SOUTH + margin, NORTH - margin))

    # Command/control centre near the real town location.
    add("CMD", "COMMAND", 79.4542, 29.3803, label="Control Centre")
    # Two ground stations (drone launch/recovery).
    add("BASE_A", "BASE", 79.36, 29.30, label="Ground Station A", speed_kmh=55)
    add("BASE_B", "BASE", 79.55, 29.45, label="Ground Station B", speed_kmh=55)

    # Eight survey zones with a monitoring priority (1-10): landslide/flood risk.
    for i in range(1, 9):
        add(f"Z{i}", "ZONE", rand_lon(), rand_lat(),
            label=f"Survey Zone {i}",
            priority=round(float(rng.uniform(2.0, 8.0)), 1))

    # Ten survey checkpoints (waypoints flown on the routine circuit).
    for i in range(1, 11):
        add(f"C{i}", "CHECKPOINT", rand_lon(), rand_lat(),
            label=f"Checkpoint {i}")

    # ~32 comms relays, biased toward high ground (realistic for radio relays).
    placed = 0
    target = 32
    attempts = 0
    while placed < target and attempts < 5000:
        attempts += 1
        lon, lat = rand_lon(0.01), rand_lat(0.01)
        e = _sample(elev, lon, lat)
        # accept high ground more often
        if rng.uniform(0, 1) < (e - ELEV_MIN) / (ELEV_MAX - ELEV_MIN) + 0.15:
            placed += 1
            add(f"R{placed}", "RELAY", lon, lat,
                label=f"Relay {placed}", antenna_h_m=30.0,
                activation_cost=round(float(rng.uniform(8, 20)), 1))

    return {"type": "FeatureCollection",
            "crs": {"type": "name",
                    "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}},
            "features": feats}


def build_nofly() -> dict:
    """Two restricted polygons (e.g. protected wildlife core + a buffer)."""
    def rect(w, s, e, n, name):
        return {"type": "Feature",
                "properties": {"name": name},
                "geometry": {"type": "Polygon",
                             "coordinates": [[[w, s], [e, s], [e, n], [w, n], [w, s]]]}}
    feats = [
        rect(79.40, 29.31, 79.45, 29.35, "Protected Core Area"),
        rect(79.50, 29.38, 79.535, 29.41, "Restricted Buffer"),
    ]
    return {"type": "FeatureCollection",
            "crs": {"type": "name",
                    "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}},
            "features": feats}


def main():
    elev = build_dem()
    dem_path = write_dem(elev)
    feats = build_features(elev)
    nofly = build_nofly()
    (DATA / "features.geojson").write_text(json.dumps(feats, indent=1))
    (DATA / "nofly.geojson").write_text(json.dumps(nofly, indent=1))

    kinds: dict[str, int] = {}
    for f in feats["features"]:
        kinds[f["properties"]["kind"]] = kinds.get(f["properties"]["kind"], 0) + 1
    print(f"DEM        : {dem_path}  ({DEM_W}x{DEM_H}, "
          f"{elev.min():.0f}-{elev.max():.0f} m, WGS84)")
    print(f"Bounds     : {WEST},{SOUTH} .. {EAST},{NORTH}")
    print(f"Features   : {sum(kinds.values())}  {kinds}")
    print(f"No-fly     : {len(nofly['features'])} polygons")


if __name__ == "__main__":
    main()
