import os
from fastapi import FastAPI, Depends, HTTPException, Request
from pydantic import BaseModel
import uvicorn
from pyngrok import ngrok, conf
import secrets
import time
import subprocess
import threading

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

GPS_Terr_Check_Activate = False #False to waive GPS and terrain checks in start server (For testing purposes only)

try:
    oled = OLEDDisplay(i2c_port=1, i2c_address=0x3C)
    import atexit
    atexit.register(oled.clear)
except Exception as e:
    print(f"[OLED] Hardware not available: {e} — desktop QR only")
    oled = OLEDDisplay.__new__(OLEDDisplay)

def check_internet():
    try:
        # Check connectivity silently
        result = subprocess.run(['ping', '-c', '1', '-W', '5', '8.8.8.8'], capture_output=True, timeout=10)
        return result.returncode == 0
    except:
        return False
        
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
                if not node_ref.prearm_failed:
                    try:
                        oled.show_battery(node_ref.battery_percentage)
                    except:
                        pass
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
        "prearm_failed": node_ref.prearm_failed,
        "prearm_message": node_ref.prearm_message,
    }

@app.post("/set_global_target")
def set_global_target(target: GlobalTarget, key: str = Depends(auth)):
    print("[DEBUG] Set global target Called")
    global pending_waypoints
    if not node_ref:
        return {"error": "Node not running"}
        node_ref.get_logger().info("Node not running")
    pending_waypoints.append((target.lat, target.lon, target.alt))
    node_ref.get_logger().info(f"Waypoint loaded: {(target.lat, target.lon, target.alt)}")
    return {"status": "Waypoint queued", "total": len(pending_waypoints)}

@app.post("/clear_waypoints")
def clear_waypoints(key: str = Depends(auth)):
    print("[DEBUG] Clear Waypoints Called")
    global pending_waypoints, node_ref
    pending_waypoints = []
    if node_ref:
        node_ref.clear_targets()
    return {"status": "Waypoints cleared"}

@app.post("/start_mission")
def start_mission(takeoff_alt: float = 10.0, key: str = Depends(auth)):
    print("[DEBUG] Start Mission Called")
    global node_ref, pending_waypoints

    if not node_ref:
        node_ref.get_logger().error("Node Not Running")
        return {"error": "Node not running"}
    if not node_ref.terrain_loaded:
        return {"error": f"Terrain data not ready — {node_ref.terrain_pending} tiles pending"}
    if not pending_waypoints:
        node_ref.get_logger().error("No Waypoints Queued")
        return {"error": "No waypoints queued"}
    if node_ref.home_lat is None:
        node_ref.get_logger().error("Home GPS not yet acquired")
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
    
def has_gps_fix(node):
    return (node.current_lat is not None and node.current_lat != 0.0 and node.current_lon != 0.0)

def get_ngrok_token():

    # Try environment variable first
    
    token = os.getenv("NGROK_AUTH_TOKEN")
    if token:
        return token
    
    # Fall back to reading ngrok config file directly
    
    config_paths = [
        os.path.expanduser("/home/chatwil683/snap/ngrok/425/.config/ngrok/ngrok.yml"),
        os.path.expanduser("/home/chatwil683/snap/ngrok/430/.config/ngrok/ngrok.yml"),
    ]
    
    for path in config_paths:
        if os.path.exists(path):
            with open(path) as f:
                for line in f:
                    if "authtoken" in line:
                        print("Authtoken loaded from Config File")
                        oled.show_status("Authtoken loaded from Config File")
                        time.sleep(1)
                        return line.split(":")[-1].strip()
    oled.show_status("No paths loaded ngrok token")
    time.sleep(2)
    return None

def start_lte_tunnel(port=8000, retries=5):
    global binding_url
    
    #wait for GPS Fix
    print("[GPS] Waiting for GPS Fix")
    try:
        oled.show_status("Waiting for GPS Fix")
    except:
        pass
    
    while node_ref is None or not has_gps_fix(node_ref) and GPS_Terr_Check_Activate:
        time.sleep(1)
        
    
    print("[GPS] GPS Fix acquired")
    
    # Wait for terrain
    print("[TERRAIN] Waiting for terrain data...")
    try:
        oled.show_status("Loading terrain...")
    except:
        pass
    
    while node_ref is None or not node_ref.terrain_loaded and GPS_Terr_Check_Activate:
        time.sleep(1)
    
    print("[TERRAIN] Terrain Loaded")
    
    auth_token = get_ngrok_token()
    
    if auth_token:
        oled.show_status(f"Auth token found: {auth_token[:8]}...")
        print(f"Auth token found: {auth_token[:8]}...")
        time.sleep(2)
        oled.show_status("Starting connection to Ngrok, will require connection to internet through PPP first")
        time.sleep(2)
    else:
        oled.show_status("Auth_token not loaded from bashRC")
        print("Auth_token not loaded from bashRC")
        
    
    conf.get_default().auth_token = auth_token
    conf.get_default().region = "eu"
    conf.get_default().update_check = False
    
    # Connecting to ngrok
    for attempt in range(retries):
        
        #Internet Check
        while not check_internet():
            print(f"[TUNNEL] No Internet - waiting for PPP to connect...")
            time.sleep(1)
            try:
                oled.show_status("No Signal....")
                retries = 5
            except:
                pass
        
        print(f"[TUNNEL] Internet OK - connecting ngrok (attempt {attempt + 1})...")
        oled.show_status(f"Service OK, attempting Ngrok Connection, attempt {attempt + 1}")
        time.sleep(2)
        
        try:
            url = ngrok.connect(port)
            binding_url = url.public_url # just the clean https URL
            print(f"[TUNNEL] Public URL: {binding_url}")
            oled.show_status("NGROK Connected")
            time.sleep(1)
            
            try:
                oled.show_status("Generating QR....")
                time.sleep(2)
                oled.show_qr_desktop(binding_url, API_KEY)  # always works — desktop and Pi
                oled.show_qr(binding_url, API_KEY)           # only works on Pi with OLED hardware
            except Exception as e:
                print(f"[OLED] Failed: {e}")
            
            while True:
                time.sleep(15)
                if not ngrok.get_tunnels():
                    print("[TUNNEL] Tunnel dropped - reconnecting...")
                    oled.show_status("Tunnel dropped, reconnecting for new QR in 5 seconds")
                    ngrok.kill()
                    retries = 5
                    time.sleep(5)
                    break
            
        except Exception as e:
            print(f"[TUNNEL] Attempt {attempt+1} failed: {e}")
            oled.show_status(f"Ngrok attempt {attempt+1} of {retries} failed")
            if attempt < retries -1:
                print(f"[Tunnel] Retrying in 5 seconds...")
                time.sleep(5)
    

def start_server(node: OffboardControl, host="0.0.0.0", port=8000):
    global node_ref
    node_ref = node
    
    #So Offboard Control can access oled, doesnt allow control unless app is connected, i.e doesn't interrupt QR codes
    node.oled_callback = lambda msg: oled.show_status(msg) if _app_connected else None
    
    threading.Thread(target=start_lte_tunnel, args = (port,), daemon=True).start()
    node.get_logger().info(f"[WEB] FastAPI running on {host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level="info")
    
