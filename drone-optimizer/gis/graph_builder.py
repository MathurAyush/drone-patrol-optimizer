"""
graph_builder.py
================
Turns the real GIS scene into the routing graph the C++ engine consumes.

Two edge layers, both grounded in real geography:

  FLIGHT_PATH  between BASE / ZONE / CHECKPOINT / COMMAND nodes:
      weight (effective_cost) = geodesic horizontal distance + terrain climb
      energy (from the DEM). Edges that enter restricted airspace, or exceed the
      platform's range, are flagged no_fly and the engine drops them.

  RADIO_LINK   between RELAY / COMMAND nodes:
      exists only if within radio range AND the DEM terrain profile does not
      block line-of-sight between the antennas. link_cost = slant distance.

Node x/y are local UTM kilometres (conformal, 1 unit ~ 1 km) so the engine's
Euclidean A* heuristic remains an admissible lower bound on effective_cost
(which is always >= horizontal distance).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

from .ingest import Scene, Feature, load_scene
from .terrain import Terrain
from .airspace import Airspace
from .geodesy import geodesic_m, slant_distance_m


@dataclass
class BuildParams:
    flight_knn: int = 6          # connect each flight node to k nearest
    flight_range_km: float = 40.0
    wind_factor: float = 0.10    # uniform headwind proxy on horizontal cost
    radio_range_km: float = 12.0  # max relay hop before LOS even considered
    radio_knn: int = 10


def _project_km(scene: Scene) -> dict[str, tuple[float, float]]:
    """Project every feature to local UTM, then shift to a local km origin."""
    xy = {}
    for f in scene.features:
        x, y = scene.projector.to_metres(f.lon, f.lat)
        xy[f.id] = (x, y)
    minx = min(v[0] for v in xy.values())
    miny = min(v[1] for v in xy.values())
    return {k: ((x - minx) / 1000.0, (y - miny) / 1000.0)
            for k, (x, y) in xy.items()}


def _knn(src: Feature, others: list[Feature], k: int) -> list[Feature]:
    ranked = sorted(others, key=lambda o: geodesic_m(src.lon, src.lat, o.lon, o.lat))
    return ranked[:k]


def build_graph(scene: Scene, terrain: Terrain, airspace: Airspace,
                params: BuildParams = BuildParams()) -> dict:
    xy = _project_km(scene)
    nodes_out = []
    for f in scene.features:
        x, y = xy[f.id]
        node = {"id": f.id, "kind": f.kind, "x": round(x, 4), "y": round(y, 4),
                "label": f.props.get("label", f.id),
                "lon": f.lon, "lat": f.lat,
                "elevation_m": round(terrain.elevation(f.lon, f.lat), 1)}
        if f.kind == "ZONE":
            node["threat_score"] = float(f.props.get("priority", 1.0))
        if f.kind == "RELAY":
            node["activation_cost"] = float(f.props.get("activation_cost", 10.0))
            node["antenna_h_m"] = float(f.props.get("antenna_h_m", 15.0))
        if f.kind == "BASE":
            node["speed_kmh"] = float(f.props.get("speed_kmh", 55.0))
        nodes_out.append(node)

    edges_out = []
    seen: set[tuple[str, str]] = set()

    # --- FLIGHT layer ---------------------------------------------------
    flight_nodes = scene.of_kind("BASE", "ZONE", "CHECKPOINT", "COMMAND")
    for f in flight_nodes:
        for g in _knn(f, [o for o in flight_nodes if o.id != f.id],
                      params.flight_knn):
            key = tuple(sorted((f.id, g.id)))
            if key in seen:
                continue
            seen.add(key)
            c = terrain.climb_cost_km(f.lon, f.lat, g.lon, g.lat)
            horiz = c["horizontal_km"]
            blocked_zone = airspace.blocking_zone(f.lon, f.lat, g.lon, g.lat)
            over_range = horiz > params.flight_range_km
            no_fly = bool(blocked_zone) or over_range
            eff = c["effective_km"] * (1.0 + params.wind_factor)
            edge = {"u": f.id, "v": g.id, "kind": "FLIGHT_PATH",
                    "distance_km": round(horiz, 4),
                    "wind_resistance": params.wind_factor,
                    "no_fly": no_fly,
                    "effective_cost": round(eff, 4),
                    "climb_m": round(c["climb_m"], 1),
                    "slope": round(c["slope"], 4)}
            if blocked_zone:
                edge["blocked_by"] = blocked_zone
            edges_out.append(edge)

    # --- RADIO layer (terrain line-of-sight) ----------------------------
    relay_nodes = scene.of_kind("RELAY", "COMMAND")
    los_pass = los_fail = 0
    for f in relay_nodes:
        h1 = float(f.props.get("antenna_h_m", 15.0))
        for g in _knn(f, [o for o in relay_nodes if o.id != f.id],
                      params.radio_knn):
            key = tuple(sorted((f.id, g.id)))
            if key in seen:
                continue
            horiz_m = geodesic_m(f.lon, f.lat, g.lon, g.lat)
            if horiz_m / 1000.0 > params.radio_range_km:
                continue
            seen.add(key)
            h2 = float(g.props.get("antenna_h_m", 15.0))
            clear = terrain.has_line_of_sight(f.lon, f.lat, h1,
                                              g.lon, g.lat, h2)
            if not clear:
                los_fail += 1
                continue
            los_pass += 1
            e1 = terrain.elevation(f.lon, f.lat)
            e2 = terrain.elevation(g.lon, g.lat)
            slant_km = slant_distance_m(horiz_m, e1, e2) / 1000.0
            edges_out.append({"u": f.id, "v": g.id, "kind": "RADIO_LINK",
                              "distance_km": round(horiz_m / 1000.0, 4),
                              "link_cost": round(slant_km, 4)})

    meta = {
        "name": "uttarakhand-survey-sector",
        "crs_features": "EPSG:4326",
        "utm_epsg": scene.projector.epsg,
        "dem_path": terrain.path,
        "nofly_polygons": airspace.count,
        "radio_los_pass": los_pass,
        "radio_los_blocked": los_fail,
        "node_count": len(nodes_out),
        "edge_count": len(edges_out),
    }
    return {"meta": meta, "nodes": nodes_out, "edges": edges_out}


def run(features_path: str, dem_path: str, nofly_path: str,
        out_path: str, params: BuildParams = BuildParams()) -> dict:
    scene = load_scene(features_path)
    terrain = Terrain(dem_path)
    airspace = Airspace(nofly_path)
    graph = build_graph(scene, terrain, airspace, params)
    with open(out_path, "w") as fh:
        json.dump(graph, fh, indent=1)
    terrain.close()
    return graph["meta"]
