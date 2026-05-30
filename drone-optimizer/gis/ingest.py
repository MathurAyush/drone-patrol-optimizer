"""
ingest.py
=========
Loads and validates the real vector layer (monitoring stations, survey zones,
checkpoints, relays) from GeoJSON, and establishes the local metric projection
for the study area. This is the boundary where untrusted external geodata is
checked before it reaches the routing core.
"""

from __future__ import annotations

from dataclasses import dataclass

import geopandas as gpd

from .geodesy import LocalProjector

VALID_KINDS = {"COMMAND", "BASE", "ZONE", "CHECKPOINT", "RELAY"}


@dataclass
class Feature:
    id: str
    kind: str
    lon: float
    lat: float
    props: dict


@dataclass
class Scene:
    features: list[Feature]
    projector: LocalProjector

    def of_kind(self, *kinds: str) -> list[Feature]:
        return [f for f in self.features if f.kind in kinds]


def load_scene(features_path: str) -> Scene:
    """Read features.geojson, validate, and build the local UTM projector."""
    gdf = gpd.read_file(features_path)
    if gdf.crs is None:
        raise ValueError("features file has no CRS; expected WGS84 (EPSG:4326)")
    if gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)

    feats: list[Feature] = []
    for _, row in gdf.iterrows():
        geom = row.geometry
        if geom is None or geom.geom_type != "Point":
            raise ValueError(f"feature {row.get('id')} is not a Point geometry")
        kind = row.get("kind")
        if kind not in VALID_KINDS:
            raise ValueError(f"feature {row.get('id')} has invalid kind {kind!r}")
        props = {k: row[k] for k in gdf.columns if k != "geometry"
                 and row[k] is not None and not _is_nan(row[k])}
        feats.append(Feature(id=str(row["id"]), kind=kind,
                             lon=float(geom.x), lat=float(geom.y), props=props))

    if not feats:
        raise ValueError("no features loaded")

    # Local projection from the centroid of all features.
    clon = sum(f.lon for f in feats) / len(feats)
    clat = sum(f.lat for f in feats) / len(feats)
    proj = LocalProjector.for_point(clon, clat)
    return Scene(features=feats, projector=proj)


def _is_nan(v) -> bool:
    try:
        return v != v
    except Exception:
        return False
