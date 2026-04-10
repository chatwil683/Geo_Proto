import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome
#from geo_proto.indicator_led import IndicatorLED


class OffboardControl(Node):
    """
    ROS2 Jazzy node for controlling an ArduPilot SITL vehicle via MAVROS2.

    Features:
    - Proper GUIDED + ARM + TAKEOFF sequence (no offboard setpoints during takeoff)
    - After airborne, streams /mavros/setpoint_position/local at 10 Hz
    - Accepts both:
        * Local ENU setpoints (x, y, z)
        * Absolute GPS waypoints (lat, lon, alt) via set_global_target()
    """

    def __init__(self):
        super().__init__('offboard_control')

        # --- Publishers & subscribers ---
        self.pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            qos_profile_sensor_data
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            10
        )

        self.global_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_callback,
            qos_profile_sensor_data
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )       
        
        
        # --- MAVROS services ---
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.home_cli = self.create_client(CommandHome, '/mavros/cmd/set_home')

        # --- Internal state ---
        # Local ENU targets (x, y, z) in 'map' frame
        self.targets = []                # list of (x, y, z)
        self.current_idx = -1
        self.current_setpoint = (0.0, 0.0, 10.0)  # default hover 10 m up

        self.state: State | None = None
        self.local_pose: PoseStamped | None = None

        # Home GPS (lat/lon/alt) taken from first global position (or refine later)
        self.home_lat: float | None = None
        self.home_lon: float | None = None
        self.home_alt: float | None = None
        

        # Timer for streaming offboard setpoints (created AFTER takeoff)
        self.timer = None

        # --- LED helper ---
        #self.led = IndicatorLED(line_offset=17)
        #self.led.set_state("boot")  # slow blink on node start
        #self.get_logger().info("[LED] Indicator LED initialised.")

        self.get_logger().info("OffboardControl initialised.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def state_callback(self, msg: State):
        """Monitor FCU state (mode, armed, etc.)."""
        self.state = msg

        #if not msg.connected:
            #self.led.set_state("disconnected")
        #elif not msg.armed:
            #self.led.set_state("idle")
        # else: leave LED state for arm/takeoff/offboard control

    def local_pose_callback(self, msg: PoseStamped):
        """Track local position to know when we're airborne."""
        self.local_pose = msg

    def global_callback(self, msg: NavSatFix):
        """
        Track GPS; use first fix as 'home' unless already set.
        You can swap this to /mavros/home_position/home if preferred.
        """
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.get_logger().info(
                f"[GLOBAL] Home set from GPS: "
                f"lat={self.home_lat:.7f}, lon={self.home_lon:.7f}, alt={self.home_alt:.1f}"
            )

    # ------------------------------------------------------------------
    # Target management (local ENU)
    # ------------------------------------------------------------------

    def set_target(self, x: float, y: float, z: float):
        self.targets.append((x, y, z))  # ✅ append instead of replace

        # If there’s no active target, start with the first one
        if self.current_idx == -1:
            self.current_idx = 0
            self.current_setpoint = self.targets[0]

        self.get_logger().warn(f"[APP] NEW TARGET QUEUED: ({x:.2f}, {y:.2f}, {z:.2f})")
        
    def clear_targets(self):
        """Clear queued targets."""
        self.targets = []
        self.current_idx = -1
        self.get_logger().info("[TARGET] Targets cleared.")

    def next_target(self):
        """Cycle to next target (placeholder for future multi-waypoint logic)."""
        if not self.targets:
            return
        self.current_idx = (self.current_idx + 1) % len(self.targets)
        self.current_setpoint = self.targets[self.current_idx]
        self.get_logger().info(f"[TARGET] Switching to {self.current_setpoint}")

    # ------------------------------------------------------------------
    # Local ENU <-> GPS helpers
    # ------------------------------------------------------------------

    def gps_to_local(self, lat: float, lon: float, alt: float):
        """Convert WGS84 lat/lon/alt to local ENU meters relative to home."""
        if self.home_lat is None or self.home_lon is None or self.home_alt is None:
            self.get_logger().warning("[GLOBAL] Home GPS not set yet; cannot convert to local.")
            return None

        # Earth radius in meters (approximate)
        R = 6378137.0

        d_lat = math.radians(lat - self.home_lat)
        d_lon = math.radians(lon - self.home_lon)

        x = R * d_lon * math.cos(math.radians(self.home_lat))  # East
        y = R * d_lat                                          # North
        z = alt                            # Up relative to home

        return (x, y, z)

    def set_global_target(self, lat: float, lon: float, alt: float):
        """Set target using absolute GPS coordinates."""
        local = self.gps_to_local(lat, lon, alt)
        if local is None:
            self.get_logger().error("[GLOBAL] Cannot set global target; no home yet.")
            return

        x, y, z = local
        self.set_target(x, y, z)
        self.get_logger().info(
            f"[GLOBAL] Global target lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f} "
            f"-> local ({x:.1f}, {y:.1f}, {z:.1f})"
        )
    
    def return_to_home_immediate(self):
        """Clear all targets and send the drone straight to home coordinates."""
        if self.home_lat is None:
            self.get_logger().warning("[RTH] Home not set; cannot return home.")
            return

        self.clear_targets()  # Remove any queued waypoints
        x, y, z = 0.0, 0.0, 10.0  # local ENU home at 10m altitude
        self.set_target(x, y, z)
        self.get_logger().info(f"[RTH] Returning home immediately → setpoint: ({x}, {y}, {z})")
    

    # ------------------------------------------------------------------
    # Setpoint streaming control
    # ------------------------------------------------------------------

    def start_setpoint_streaming(self):
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.publish_target)  # 10 Hz
            #self.led.set_state("offboard")  # heartbeat while offboard
            self.get_logger().info("[OFFBOARD] Setpoint streaming STARTED (10 Hz).")

    def stop_setpoint_streaming(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            #self.led.set_state("airborne")  # back to solid airborne
            self.get_logger().info("[OFFBOARD] Setpoint streaming STOPPED.")

    # ------------------------------------------------------------------
    # Arming / Mode / Takeoff
    # ------------------------------------------------------------------
    def arm_and_guided(self, takeoff_alt: float | None = None):
        """
        1. Stop setpoint streaming (don't fight the takeoff controller).
        2. Set GUIDED mode.
        3. Arm.
        4. If takeoff_alt is provided:
             - Send CommandTOL
             - Wait until altitude > threshold.
        5. Start setpoint streaming at 10 Hz.
        """

        self.get_logger().info("[ARM] Starting arm + guided sequence...")
        #self.led.set_state("arming")  # fast blink during arm/takeoff

        # --- 0) Make sure services are available ---
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode...')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming...')

        # --- 1) Stop any existing setpoint streaming during takeoff ---
        self.stop_setpoint_streaming()

        # --- 2) Set GUIDED mode ---
        req_mode = SetMode.Request()
        req_mode.custom_mode = 'GUIDED'
        future_mode = self.mode_cli.call_async(req_mode)
        while not future_mode.done():
            time.sleep(0.05)
        result_mode = future_mode.result()
        if not (result_mode and result_mode.mode_sent):
            self.get_logger().error("[ARM] Failed to set GUIDED mode ❌")
            return
        self.get_logger().info("[ARM] GUIDED mode set ✅")

        # --- 3) Arm the vehicle ---
        req_arm = CommandBool.Request()
        req_arm.value = True
        future_arm = self.arm_cli.call_async(req_arm)
        while not future_arm.done():
            time.sleep(0.05)
        result_arm = future_arm.result()
        if not (result_arm and result_arm.success):
            self.get_logger().error("[ARM] Arming failed ❌")
            return
        self.get_logger().info("[ARM] Vehicle armed ✅")

        # --- 4) Takeoff (optional) ---
        if takeoff_alt is not None:
            while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /mavros/cmd/takeoff...')

            self.get_logger().info(f"[TAKEOFF] Requesting takeoff to {takeoff_alt:.1f} m...")
            req_to = CommandTOL.Request()
            req_to.altitude = float(takeoff_alt)
            req_to.latitude = 0.0   # use current location
            req_to.longitude = 0.0
            req_to.min_pitch = 0.0
            req_to.yaw = 0.0

            future_to = self.takeoff_cli.call_async(req_to)
            while not future_to.done():
                time.sleep(0.05)
            self.get_logger().info("[TAKEOFF] Takeoff command sent (CommandTOL).")

            # --- Wait until we are actually airborne ---
            target_threshold = max(1.5, takeoff_alt * 0.3)  # ≥1.5 m or 30% of target
            self.get_logger().info(f"[TAKEOFF] Waiting for altitude > {target_threshold:.1f} m...")
            start_wait = time.time()
            timeout = 20.0  # seconds

            while time.time() - start_wait < timeout:
                if self.local_pose is not None:
                    z = self.local_pose.pose.position.z
                    if z > target_threshold:
                        self.get_logger().info(f"[TAKEOFF] Airborne at altitude {z:.2f} m ✅")
                        #self.led.set_state("airborne")  # solid ON
                        break
                time.sleep(0.1)
            else:
                self.get_logger().warning("[TAKEOFF] Timeout waiting for altitude to increase.")
        else:
            self.get_logger().info("[ARM] No takeoff_alt provided; skipping takeoff step.")

        # --- 5) Start streaming offboard setpoints ---
        self.start_setpoint_streaming()
    # ------------------------------------------------------------------
    # Continuous setpoint streaming
    # ------------------------------------------------------------------

    def publish_target(self):

        base_x, base_y, base_z = self.current_setpoint
        t = time.time()
        radius = 0.05

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = base_x + radius * math.sin(t)
        msg.pose.position.y = base_y + radius * math.cos(t)
        msg.pose.position.z = base_z
        msg.pose.orientation.w = 1.0
        self.pos_pub.publish(msg)

        # Check if reached current target
        if self.local_pose and self.targets:
            dx = self.local_pose.pose.position.x - base_x
            dy = self.local_pose.pose.position.y - base_y
            dz = self.local_pose.pose.position.z - base_z
            distance = (dx**2 + dy**2 + dz**2)**0.5

            if distance < 0.3:  # 30 cm threshold
                if self.current_idx + 1 < len(self.targets):
                    self.current_idx += 1
                    self.current_setpoint = self.targets[self.current_idx]
                    self.get_logger().info(f"[NAV] Moving to next waypoint: {self.current_setpoint}")
                else:
                    self.get_logger().info("[NAV] Final waypoint reached.")
    
                    # Clear targets so we don't retrigger
                    self.targets = []
                    self.current_idx = -1

                    # Initiate landing
                    self.initiate_landing()
                    return
                    
    def set_home_here(self):
        self.get_logger().info("[HOME] Setting current position as new home...")

        req = CommandHome.Request()
        req.current_gps = True

        future = self.home_cli.call_async(req)

        def _response_callback(fut):
            try:
                result = fut.result()
                if result and result.success:
                    self.get_logger().info("[HOME] Home position updated ✅")
                else:
                    self.get_logger().error("[HOME] Failed to set home ❌")
            except Exception as e:
                self.get_logger().error(f"[HOME] Service call failed: {e}")

        future.add_done_callback(_response_callback)
            
    def initiate_landing(self):
        self.get_logger().info("[LAND] Initiating landing sequence...")

        req = SetMode.Request()
        req.custom_mode = "LAND"

        future = self.mode_cli.call_async(req)
        future.add_done_callback(self._land_mode_response)
        
    def _land_mode_response(self, future):
        result = future.result()

        if result and result.mode_sent:
            self.get_logger().info("[LAND] LAND mode activated ✅")

            # Start landing monitor
            self.stable_alt_count = 0
            self.prev_alt = None

            # Create landing monitor timer (10 Hz)
            self.landing_timer = self.create_timer(0.1, self._landing_monitor)

        else:
            self.get_logger().error("[LAND] Failed to switch to LAND mode ❌")
    
    def _landing_monitor(self):
        if self.local_pose is None or self.state is None:
            self.get_logger().info("stuck in check")
            return

        alt = self.local_pose.pose.position.z
        self.get_logger().info(f"[LAND] Altitude: {alt:.2f}")

        if self.prev_alt is not None and abs(alt - self.prev_alt) < 0.05:
            self.stable_alt_count += 1
        else:
            self.stable_alt_count = 0

        self.prev_alt = alt

        # 5 seconds stable at 10Hz = 50 counts
        if not self.state.armed and self.stable_alt_count >= 50:
            self.get_logger().info("[LAND] Drone landed and stable ✅")

            # Stop this timer
            self.landing_timer.cancel()

            # Switch to GUIDED to set home
            req = SetMode.Request()
            req.custom_mode = "GUIDED"
            future = self.mode_cli.call_async(req)
            future.add_done_callback(self._guided_after_land)
            
    def _guided_after_land(self, future):
        result = future.result()

        if result and result.mode_sent:
            self.get_logger().info("[LAND] Switched to GUIDED for home reset ✅")
        else:
            self.get_logger().warn("[LAND] GUIDED switch failed, continuing anyway")

        self.set_home_here()
        self.get_logger().info("[LAND] Landing sequence complete; home updated ✅")
        
        # Reset internal state so we can arm & guided again
        self.reset_after_landing()
        
    def reset_after_landing(self):
        """Reset internal state after landing to allow re-arming and takeoff."""
        self.get_logger().info("[RESET] Resetting state for next flight.")

        # Stop any timers
        if hasattr(self, 'landing_timer') and self.landing_timer is not None:
            self.landing_timer.cancel()
            self.landing_timer = None

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        # Clear targets and setpoint index
        self.targets = []
        self.current_idx = -1
        self.current_setpoint = (0.0, 0.0, 10.0)

        # Reset landing counters
        self.stable_alt_count = 0
        self.prev_alt = None

        # Drone should still be in GUIDED, disarmed
        self.get_logger().info("[RESET] Internal state cleared. Ready to arm and takeoff.")
        


            
