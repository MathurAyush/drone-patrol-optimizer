"""
api_server.py
=============
Flask REST API in front of the Scheduler. Serves the tactical dashboard and
exposes one endpoint per operational question. CORS-open for local dev.

Run:  python python_scheduler/api_server.py
Then open http://127.0.0.1:5000/
"""

from __future__ import annotations

import json
from pathlib import Path

from flask import Flask, jsonify, request, Response, send_from_directory

from scheduler import Scheduler, EngineError, DEFAULT_GRAPH

REPO_ROOT = Path(__file__).resolve().parents[1]
DASHBOARD_DIR = REPO_ROOT / "dashboard"

app = Flask(__name__)
scheduler = Scheduler()


@app.after_request
def add_cors(resp: Response) -> Response:
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp


def ok(data):
    return jsonify({"ok": True, "data": data})


def fail(msg, code=400):
    return jsonify({"ok": False, "error": str(msg)}), code


@app.get("/")
def index():
    return send_from_directory(DASHBOARD_DIR, "index.html")


@app.get("/api/graph")
def api_graph():
    """The full zone graph for the dashboard to render."""
    return ok(json.loads(Path(DEFAULT_GRAPH).read_text()))


@app.get("/api/fleet")
def api_fleet():
    return ok([d.to_payload() for d in scheduler.fleet])


@app.get("/api/demand")
def api_demand():
    return ok(scheduler.demand_table())


@app.get("/api/patrol")
def api_patrol():
    """Routine patrol circuit. ?exact=false for the NN heuristic."""
    exact = request.args.get("exact", "true").lower() != "false"
    try:
        return ok(scheduler.patrol_circuit(exact=exact))
    except EngineError as e:
        return fail(e, 500)


@app.get("/api/dispatch")
def api_dispatch():
    """Fastest-drone dispatch to ?zone=Z7 (&method=astar|dijkstra)."""
    zone = request.args.get("zone")
    method = request.args.get("method", "astar")
    if not zone:
        return fail("missing ?zone=")
    try:
        return ok(scheduler.dispatch(zone, method=method))
    except EngineError as e:
        return fail(e, 500)


@app.get("/api/allocate")
def api_allocate():
    """Max-flow flight-hour allocation across all zones."""
    try:
        return ok(scheduler.allocate())
    except EngineError as e:
        return fail(e, 500)


@app.get("/api/relay")
def api_relay():
    """Relay MST backbone. ?failed=R5 to simulate a tower failure."""
    failed = request.args.get("failed", "")
    try:
        return ok(scheduler.relay_backbone(failed_relay=failed))
    except EngineError as e:
        return fail(e, 500)


@app.get("/api/fleet_routes")
def api_fleet_routes():
    """Multi-drone VRP routing. ?drones=3&range_km=60"""
    drones = int(request.args.get("drones", 3))
    range_km = request.args.get("range_km", type=float)
    try:
        return ok(scheduler.multi_drone_patrol(num_drones=drones,
                                               range_km=range_km))
    except Exception as e:
        return fail(e, 500)


@app.get("/api/health")
def api_health():
    return ok({"status": "up", "engine_graph": DEFAULT_GRAPH})


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)
