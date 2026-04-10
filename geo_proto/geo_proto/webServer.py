# geo_proto_lte_api.py
from fastapi import FastAPI, Depends, HTTPException
from pydantic import BaseModel
import uvicorn
from pyngrok import ngrok

from geo_proto.offboardControl import OffboardControl
from geo_proto.QrOut import OLEDDisplay

# ---------- FastAPI App ----------
app = FastAPI(title="GeoProto Drone API LTE", version="1.0.0")

# ---------- Global References ----------
node_ref: OffboardControl | None = None
binding_url: str | None = None
API_KEY = "supersecret"  # <-- change this to a secure key
oled = OLEDDisplay(i2c_port=1, i2c_address=0x3C)  # Initialize OLED

# ---------- Authentication ----------
def auth(key: str):
    if key != API_KEY:
        raise HTTPException(status_code=401, detail="Unauthorized")

# ---------- Data Models ----------
class Target(BaseModel):
    x: float
    y: float
    z: float

class GlobalTarget(BaseModel):
    lat: float
    lon: float
    alt: float

# ---------- Endpoints ----------
@app.get("/")
def root():
    return {"message": "GeoProto Drone API LTE online ✅"}

@app.get("/binding")
def get_binding_url():
    """Return the public LTE binding URL for the iOS app"""
    if not binding_url:
        return {"error": "Binding not ready yet"}
    return {"url": binding_url}

@app.get("/status")
def get_status(api_key: str = Depends(auth)):
    global node_ref
    if not node_ref or not node_ref.state:
        return {"connected": False, "armed": False, "mode": "UNKNOWN"}

    s = node_ref.state
    x, y, z = node_ref.current_setpoint

    return {
        "connected": True,
        "armed": bool(s.armed),
        "mode": s.mode,
        "target": {"x": x, "y": y, "z": z},
        "targets": [{"x": t[0], "y": t[1], "z": t[2]} for t in node_ref.targets],
    }

@app.post("/set_target")
def set_target(target: Target, api_key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    node_ref.set_target(target.x, target.y, target.z)
    return {"status": "Target set", "target": target.dict()}

@app.post("/clear_waypoints")
def clear_waypoints(api_key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    node_ref.clear_targets()
    return {"status": "Waypoints cleared"}

@app.post("/start_mission")
def start_mission(takeoff_alt: float = 10.0, api_key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    node_ref.get_logger().info(f"[WEB] Start Mission → GUIDED + Arm + Takeoff to {takeoff_alt}m")
    node_ref.arm_and_guided(takeoff_alt=takeoff_alt)
    return {"status": "Mission started", "takeoff_alt": takeoff_alt}

@app.post("/set_global_target")
def set_global_target(target: GlobalTarget, api_key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    if node_ref.home_lat is None:
        return {"error": "Home GPS not yet acquired"}
    node_ref.set_global_target(target.lat, target.lon, target.alt)
    return {"status": "Global target set", "target": target.dict()}

@app.post("/return_home")
def return_home(api_key: str = Depends(auth)):
    global node_ref
    if not node_ref:
        return {"error": "Node not running"}
    if node_ref.home_lat is None:
        return {"error": "Home GPS not yet acquired"}
    node_ref.return_to_home_immediate()
    return {"status": "Returning home immediately"}

# ---------- LTE Tunnel Setup with OLED QR ----------
def start_lte_tunnel(port=8000):
    """
    Start ngrok tunnel, store URL globally,
    and display QR code with API key on OLED.
    """
    global binding_url, API_KEY, oled

    url = ngrok.connect(port)
    binding_url = str(url)
    print(f"[TUNNEL] Public LTE URL: {binding_url}")

    try:
        oled.show_qr(binding_url, API_KEY)
        print("[OLED] QR code displayed on screen")
    except Exception as e:
        print(f"[OLED] Failed to display QR: {e}")

    return binding_url

# ---------- Server Starter ----------
def start_server(node: OffboardControl, host="0.0.0.0", port=8000):
    """
    Start FastAPI server with reference to ROS node,
    LTE tunnel, and QR code display.
    """
    global node_ref, binding_url
    node_ref = node

    # Start LTE tunnel and show QR
    binding_url = start_lte_tunnel(port)

    node.get_logger().info(f"[WEB] FastAPI server running on {host}:{port}")
    node.get_logger().info(f"[WEB] Public LTE binding URL: {binding_url}")

    uvicorn.run(app, host=host, port=port, log_level="info")
