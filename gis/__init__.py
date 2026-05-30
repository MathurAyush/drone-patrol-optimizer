"""GIS ingestion layer: real geospatial data -> routing graph."""
from .geodesy import LocalProjector, geodesic_m, utm_epsg_for
from .terrain import Terrain
from .airspace import Airspace
