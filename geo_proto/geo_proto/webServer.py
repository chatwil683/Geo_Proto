import os
from fastapi import FastAPI, Depends, HTTPException, Request
from pydantic import BaseModel
import uvicorn
from pyngrok import ngrok, conf
import secrets

from geo_proto.offboardControl import OffboardControl
from geo_proto.QrOut import OLEDDisplay

app = FastAPI(title="GeoProto Drone API", version="2.0.0")

node_ref: OffboardControl | None = None
binding_url: str | None = None
API_KEY = secrets.token_urlsafe(16)
print(f"[AUTH] Generated API key: {API_KEY}")
pending_waypoints: list[tuple[float, float, float]] = []
_app_connected = False
_session_token: str | None = None

try:
    oled = OLEDDisplay(i2c_port=1, i2c_address=0x3C)
except Exception as e:
    print(f"[OLED] Hardware not available: {e} — desktop QR only")
    oled = OLEDDisplay.__new__(OLEDDisplay)

def auth(request: Request, key: str = "", session: str = ""):
    if key != API_KEY:
        raise HTTPException(status_code=401, detail="Unauthorized")
    if _session_token and session != _session_token:
        raise HTTPException(status_code=403, detail="Invalid session — call /handshake first")

class GlobalTarget(BaseModel):
    lat: float
    lon: float
    alt: float

@app.get("/")
def root():
    return {"message": "GeoProto Drone API online ✅"}

@app.post("/handshake")
def handshake(key: str = Depends(auth)):
    global _session_token
    _session_token = secrets.token_urlsafe(8)
    print(f"[AUTH] Session token generated: {_session_token}")
    return {"status": "Bound", "session_token": _session_token}

@app.get("/status")
def get_status(key: str = Depends(auth)):
    global node_ref, _app_connected
    
    # First time app calls /status — switch OLED from QR to battery
    if not _app_connected:
        _app_connected = True
        try:
            if node_ref and node_ref.battery_percentage is not None:
                oled.show_battery(node_ref.battery_percentage)
        except Exception as e:
            print(f"[OLED] Battery display failed: {e}")
            
    if not node_ref or not node_ref.state:
        return {"connected": False, "armed": False, "mode": "UNKNOWN"}
        
    # Update battery display on every subsequent call
    if node_ref.battery_percentage is not None:
        try:
            oled.show_battery(node_ref.battery_percentage)
        except Exception:
            pass
            
    s = node_ref.state
    
    return {
        "connected": bool(s.connected),
        "armed":     bool(s.armed),
        "mode":      s.mode,
        "target": {"lat": node_ref.targets[0][0], "lon": node_ref.targets[0][1], "alt": node_ref.targets[0][2]} if node_ref.targets else None,
        "targets": [{"lat": wp[0], "lon": wp[1], "alt": wp[2]} for wp in node_ref.targets],
        "pending_waypoints": len(pending_waypoints),
        "lat": node_ref.current_lat,
        "lon": node_ref.current_lon,
        "alt": node_ref.current_alt,
        "battery_voltage": node_ref.battery_voltage,
        "battery_percentage": node_ref.battery_percentage,
        "terrain_ready": node_ref.terrain_loaded,
        "terrain_pending": node_ref.terrain_pending,
    }

@app.post("/set_global_target")
def set_global_target(target: GlobalTarget, key: str = Depends(auth)):
    global pending_waypoints
    if not node_ref:
        return {"error": "Node not running"}
    pending_waypoints.append((target.lat, target.lon, target.alt))
    return {"status": "Waypoint queued", "total": len(pending_waypoints)}

@app.post("/clear_waypoints")
def clear_waypoints(key: str = Depends(auth)):
    global pending_waypoints, node_ref
    pending_waypoints = []
    if node_ref:
        node_ref.clear_targets()
    return {"status": "Waypoints cleared"}

@app.post("/start_mission")
def start_mission(takeoff_alt: float = 10.0, key: str = Depends(auth)):
    global node_ref, pending_waypoints

    if not node_ref:
        return {"error": "Node not running"}
    if not node_ref.terrain_loaded:
        return {"error": f"Terrain data not ready — {node_ref.terrain_pending} tiles pending"}
    if not pending_waypoints:
        return {"error": "No waypoints queued"}
    if node_ref.home_lat is None:
        return {"error": "Home GPS not yet acquired"}
    node_ref.upload_mission(pending_waypoints, takeoff_alt=takeoff_alt)
    node_ref.arm_and_start_mission(takeoff_alt=takeoff_alt)
    pending_waypoints = []
    return {"status": "Mission started", "takeoff_alt": takeoff_alt}

@app.post("/return_home")
def return_home(key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    node_ref.return_to_home_immediate()
    return {"status": "RTL activated"}

@app.post("/emergency_land")
def emergency_land(key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    node_ref.emergency_land()
    return {"status": "Emergency land activated"}

def start_lte_tunnel(port=8000):
    global binding_url
    conf.get_default().auth_token = os.getenv("NGROK_AUTH_TOKEN")
    url = ngrok.connect(port)
    binding_url = url.public_url # just the clean https URL
    print(f"[TUNNEL] Public URL: {binding_url}")
    try:
        oled.show_qr_desktop(binding_url, API_KEY)  # always works — desktop and Pi
        oled.show_qr(binding_url, API_KEY)           # only works on Pi with OLED hardware
    except Exception as e:
        print(f"[OLED] Failed: {e}")
    return binding_url

def start_server(node: OffboardControl, host="0.0.0.0", port=8000):
    global node_ref
    node_ref = node
    start_lte_tunnel(port)
    node.get_logger().info(f"[WEB] FastAPI running on {host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level="info")
