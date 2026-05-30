"""Tests for the GIS ingestion layer. Run: pytest gis/tests -q"""
import math
from pathlib import Path

import pytest

from gis.geodesy import geodesic_m, utm_epsg_for, LocalProjector, slant_distance_m
from gis.terrain import Terrain
from gis.airspace import Airspace
from gis.ingest import load_scene
from gis.graph_builder import build_graph, BuildParams

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "gis" / "data"
DATA_READY = (DATA / "dem.tif").exists() and (DATA / "features.geojson").exists()
needs_data = pytest.mark.skipif(not DATA_READY,
                                reason="run gis/generate_terrain.py first")


# --- geodesy ------------------------------------------------------------
def test_geodesic_known_distance():
    # Delhi (77.21,28.61) -> Mumbai (72.88,19.08) is ~1150-1170 km.
    d = geodesic_m(77.21, 28.61, 72.88, 19.08) / 1000.0
    assert 1130 < d < 1180


def test_geodesic_symmetry_and_zero():
    assert geodesic_m(79.4, 29.3, 79.4, 29.3) == pytest.approx(0.0, abs=1e-6)
    a = geodesic_m(79.0, 29.0, 79.5, 29.5)
    b = geodesic_m(79.5, 29.5, 79.0, 29.0)
    assert a == pytest.approx(b, rel=1e-9)


def test_utm_zone_for_uttarakhand():
    # ~79.4E, 29N -> UTM 44N -> EPSG 32644
    assert utm_epsg_for(79.4, 29.3) == 32644


def test_projector_roundtrip():
    proj = LocalProjector.for_point(79.45, 29.38)
    x, y = proj.to_metres(79.45, 29.38)
    lon, lat = proj.to_lonlat(x, y)
    assert lon == pytest.approx(79.45, abs=1e-6)
    assert lat == pytest.approx(29.38, abs=1e-6)


def test_slant_distance():
    # 3-4-5 triangle: 4 km horizontal + 3 km climb = 5 km slant.
    assert slant_distance_m(4000, 0, 3000) == pytest.approx(5000.0)


# --- terrain ------------------------------------------------------------
@needs_data
def test_elevation_in_expected_range():
    t = Terrain(str(DATA / "dem.tif"))
    e = t.elevation(79.45, 29.38)
    assert 500 < e < 2700
    t.close()


@needs_data
def test_climb_cost_never_below_horizontal():
    t = Terrain(str(DATA / "dem.tif"))
    c = t.climb_cost_km(79.35, 29.30, 79.55, 29.47)
    # effective cost must be >= horizontal (admissibility guarantee for A*)
    assert c["effective_km"] >= c["horizontal_km"] - 1e-9
    t.close()


@needs_data
def test_line_of_sight_flat_is_clear():
    t = Terrain(str(DATA / "dem.tif"))
    # Same point with tall antennas trivially has LOS.
    assert t.has_line_of_sight(79.45, 29.38, 30, 79.451, 29.381, 30)
    t.close()


# --- airspace -----------------------------------------------------------
@needs_data
def test_airspace_point_and_segment():
    a = Airspace(str(DATA / "nofly.geojson"))
    assert a.count >= 1
    # a segment fully outside should not cross
    assert not a.crosses(79.31, 29.26, 79.32, 29.27)


# --- graph builder ------------------------------------------------------
@needs_data
def test_built_graph_schema_and_admissibility():
    scene = load_scene(str(DATA / "features.geojson"))
    t = Terrain(str(DATA / "dem.tif"))
    air = Airspace(str(DATA / "nofly.geojson"))
    g = build_graph(scene, t, air, BuildParams())
    assert g["nodes"] and g["edges"]
    ids = {n["id"]: n for n in g["nodes"]}
    for e in g["edges"]:
        assert e["u"] in ids and e["v"] in ids
        if e["kind"] == "FLIGHT_PATH":
            # effective cost >= straight-line distance in projected km
            a, b = ids[e["u"]], ids[e["v"]]
            euclid = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
            assert e["effective_cost"] >= euclid - 1e-3
    t.close()
