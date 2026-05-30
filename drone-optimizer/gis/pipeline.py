"""
pipeline.py
===========
Command-line entry point for the GIS ingestion pipeline.

    python -m gis.pipeline \
        --features gis/data/features.geojson \
        --dem      gis/data/dem.tif \
        --nofly    gis/data/nofly.geojson \
        --out      datasets/zones_gis.json

Produces the engine-ready routing graph from real geospatial inputs. Any input
may be replaced by authoritative data (Copernicus/SRTM DEM, OSM-derived
features, official restricted-airspace polygons) with no code change.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .graph_builder import run, BuildParams

REPO = Path(__file__).resolve().parents[1]


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="GIS -> routing-graph pipeline")
    p.add_argument("--features", default=str(REPO / "gis/data/features.geojson"))
    p.add_argument("--dem", default=str(REPO / "gis/data/dem.tif"))
    p.add_argument("--nofly", default=str(REPO / "gis/data/nofly.geojson"))
    p.add_argument("--out", default=str(REPO / "datasets/zones_gis.json"))
    p.add_argument("--flight-knn", type=int, default=6)
    p.add_argument("--flight-range-km", type=float, default=40.0)
    p.add_argument("--wind", type=float, default=0.10)
    p.add_argument("--radio-range-km", type=float, default=12.0)
    p.add_argument("--radio-knn", type=int, default=10)
    a = p.parse_args(argv)

    params = BuildParams(flight_knn=a.flight_knn,
                         flight_range_km=a.flight_range_km,
                         wind_factor=a.wind,
                         radio_range_km=a.radio_range_km,
                         radio_knn=a.radio_knn)
    meta = run(a.features, a.dem, a.nofly, a.out, params)
    print(f"Wrote {a.out}")
    for k, v in meta.items():
        print(f"  {k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
