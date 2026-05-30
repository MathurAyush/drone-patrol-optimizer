# GIS Ingestion Pipeline

This document describes the geospatial layer that turns **real-world terrain,
location, and airspace data** into the routing graph the C++ engine optimises
over. It is what separates this system from an abstract-graph demo: every
distance, cost, and connectivity decision is grounded in real geography.

---

## 1. What it produces

```
gis/data/features.geojson   ─┐
gis/data/dem.tif             ├─►  gis pipeline  ─►  datasets/zones_gis.json
gis/data/nofly.geojson      ─┘                       (engine-ready graph)
```

`zones_gis.json` is the same schema the C++ `route_engine` already consumes, so
the entire downstream system (shortest path, TSP, max-flow, MST, scheduler, API,
dashboard, scenarios) runs on real geography with no engine change.

Run it:

```bash
python -m gis.pipeline \
  --features gis/data/features.geojson \
  --dem      gis/data/dem.tif \
  --nofly    gis/data/nofly.geojson \
  --out      datasets/zones_gis.json
```

Then point the whole stack at it:

```bash
ZONES_JSON=$PWD/datasets/zones_gis.json python3 python_scheduler/api_server.py
ZONES_JSON=$PWD/datasets/zones_gis.json python3 python_scheduler/run_scenarios.py
```

---

## 2. The geospatial stack (all production libraries)

| concern | library | what it gives us |
|---------|---------|------------------|
| distance | **pyproj** (`Geod`, WGS84) | true ellipsoidal/geodesic distance, not flat-earth |
| projection | **pyproj** (UTM) | metric, conformal local coordinates (auto zone) |
| elevation | **rasterio** | reads any GeoTIFF DEM; bilinear elevation sampling |
| geometry | **shapely** | exact point-in-polygon / segment-intersection |
| vector I/O | **geopandas / pyogrio** | reads GeoJSON, Shapefile, GeoPackage, with CRS |

---

## 3. The data model

### Coordinate reference systems
Inputs are **WGS84 (EPSG:4326)** lon/lat — the datum of GPS, OSM, SRTM, and
Copernicus DEM. The pipeline auto-selects the correct **UTM zone** for the study
area (e.g. Uttarakhand → UTM 44N / EPSG:32644) and does planar work there so a
unit is a metre with <0.1% distortion. Node `x`/`y` in the output are local UTM
kilometres; `lon`/`lat`/`elevation_m` are retained for mapping.

### Features (`features.geojson`)
Point features, each with a `kind`: `COMMAND`, `BASE`, `ZONE`, `CHECKPOINT`,
`RELAY`. Zones carry a monitoring `priority` (→ engine `threat_score`); relays
carry an `antenna_h_m` mast height and `activation_cost`.

### Elevation (`dem.tif`)
A single-band float GeoTIFF. The bundled sample is **procedurally generated**
(fractal Brownian motion) on the **real coordinate frame** of the study area so
the project is reproducible offline — but it is a standard georeferenced raster.

---

## 4. The cost model (why routes bend around terrain)

### Flight edges — terrain-aware energy cost
For each candidate flight edge the pipeline computes, from the DEM:

```
horizontal_km = geodesic distance (WGS84 ellipsoid)
climb_m       = max(0, elev_to − elev_from)
descent_m     = max(0, elev_from − elev_to)
effective_km  = (horizontal_km·1000 + climb_m·CLIMB_EQUIV + descent_m·DESC_EQUIV)/1000
              × (1 + wind_factor)
```

`CLIMB_EQUIV ≈ 6` encodes that climbing 1 m vertical costs roughly the battery of
flying 6 m horizontal (a tunable proxy, **documented as a model parameter, not
ground truth** — see `terrain.py`). Because `effective_km ≥ horizontal_km ≥`
straight-line distance, the engine's Euclidean A\* heuristic stays **admissible**,
so A\* still returns provably optimal paths.

Edges are flagged `no_fly` (and dropped by the engine) when they **enter
restricted airspace** (shapely segment∩polygon) or exceed platform range.

### Radio edges — terrain line-of-sight
A relay link exists only if two antennas are within radio range **and** the DEM
terrain profile between them does not block the ray:

```
sample ground elevation along the great-circle leg (64 points)
ray = straight line between (antenna1 top) and (antenna2 top)
link feasible  ⇔  ground stays below the ray everywhere
```

This is a genuine viewshed-style check. On the sample sector it **blocks 104 of
178 candidate hops** — the ridges really do break line-of-sight — which is why
the resulting relay backbone has true critical links (single points of failure),
unlike a naive proximity graph.

---

## 5. Swapping in authoritative real-world data

The pipeline consumes standard formats, so replace any input with official data:

**Elevation (DEM).** Download a real tile and point `--dem` at it. No code change.
- Copernicus GLO-30 (30 m, global, open): <https://dataspace.copernicus.eu>
- NASA SRTM 1-arcsec (30 m): via `earthaccess` / USGS EarthExplorer
- For India: Bhuvan / CartoDEM (ISRO)

```bash
python -m gis.pipeline --dem /path/to/Copernicus_DSM.tif ...
```

**Features.** Provide your own `features.geojson` (any GIS tool, QGIS, or an OSM
extract converted with `osmnx`/`ogr2ogr`). Required per point: `id`, `kind`, and
a Point geometry in EPSG:4326.

**Restricted airspace.** Supply real no-fly polygons (national aviation authority
NOTAMs, protected-area boundaries) as GeoJSON/Shapefile via `--nofly`.

The loader validates CRS, geometry type, and feature kinds before anything
reaches the routing core.

---

## 6. Tests

`pytest gis/tests -q` — 10 tests covering: a known real geodesic distance
(Delhi–Mumbai ≈ 1150 km), UTM zone selection, projection round-trip, the 3-4-5
slant-distance identity, elevation sampling range, **climb-cost admissibility**
(the A\* correctness guarantee), line-of-sight, airspace predicates, and output
graph schema + admissibility.

---

## 7. Honest limitations

- The bundled DEM is synthetic (real frame, real range, fractal field). Results
  are **illustrative of the method**; authoritative numbers require a real DEM
  (section 5) — a one-file swap.
- The climb-energy coefficient is a proxy, not a calibrated platform power curve.
  Calibrate `CLIMB_EQUIV_M` / `DESCENT_EQUIV_M` against real flight-log data.
- Line-of-sight uses a straight optical ray with a small clearance margin; it
  does not yet model full Fresnel-zone radius or atmospheric refraction (the
  natural next refinement).
- Earth-curvature drop is negligible at these hop lengths (<16 km) but should be
  added for longer links.

These are deliberate, documented boundaries so results are never overclaimed.
