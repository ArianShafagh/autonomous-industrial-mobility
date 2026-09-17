"""The monitoring dashboard — read only, no interaction.

    GET /            the live page (refreshes itself)
    GET /api/state   everything the page shows, as JSON (also what the thesis figures use)
    GET /health      is the monitor connected to the running system?

There is deliberately no way to command anything from here. The robot is autonomous: the AI
decides, the executor drives, and a person watches. The only intervention is `./scripts/stop.sh`
at the console, which is a physical-operator action, not a web button.

    venv/bin/python -m uvicorn robofetch_bridge.app:app --port 8000
"""
import os

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from starlette.requests import Request

WEB_DIR = os.environ.get("ROBOFETCH_WEB") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "robofetch_web", "web")
REFRESH_S = float(os.environ.get("ROBOFETCH_REFRESH_S", "2"))
# What this run was started with (set by mission.launch.py); shown in the header and /api/state.
RUN_INFO = {"model": os.environ.get("ROBOFETCH_MODEL") or "scripted plan",
            "planner": os.environ.get("ROBOFETCH_PLANNER") or "unknown"}

app = FastAPI(title="RoboFetch factory monitor")
templates = Jinja2Templates(directory=os.path.join(WEB_DIR, "templates"))
# follow_symlink: colcon's --symlink-install serves the stylesheet through a symlink, and
# StaticFiles refuses those by default (every asset 404s while the page itself still works).
app.mount("/static", StaticFiles(directory=os.path.join(WEB_DIR, "static"), follow_symlink=True),
          name="static")

_ros = None


def ros():
    """The ROS link is created on first use, so the app imports fine without a running system."""
    global _ros
    if _ros is None:
        from robofetch_bridge.ros_link import RosThread
        _ros = RosThread()
    return _ros


@app.get("/health")
def health():
    snapshot = ros().snapshot()
    return {"ok": True, "connected": snapshot["connected"],
            "sections": len(snapshot["sections"]),
            "decisions_seen": len(snapshot["decisions"])}


@app.get("/api/state")
def api_state():
    return JSONResponse({**ros().snapshot(), "run": RUN_INFO})


@app.get("/", response_class=HTMLResponse)
def dashboard(request: Request):
    snapshot = ros().snapshot()
    style = os.path.join(WEB_DIR, "static", "style.css")
    version = int(os.path.getmtime(style)) if os.path.exists(style) else 0
    # Starlette's current signature is (request, name, context); passing the name first makes it
    # treat the context dict as the template name ("unhashable type: dict").
    return templates.TemplateResponse(request, "dashboard.html", {
        "s": snapshot, "run": RUN_INFO, "refresh_s": REFRESH_S, "style_version": version})


def main():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("ROBOFETCH_WEB_PORT", 8000)))


if __name__ == "__main__":
    main()
