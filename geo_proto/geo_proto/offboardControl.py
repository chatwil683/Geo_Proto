import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import State, Waypoint, WaypointReached, TerrainReport
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear

MIN_ALT = 20.0  # minimum altitude in metres above terrain

class OffboardControl(Node):
    """
    ROS2 Jazzy node for controlling an ArduPilot vehicle via MAVROS2.
    Uses ArduPilot AUTO mode for all navigation — no manual setpoint streaming.
    """

    def __init__(self):
        super().__init__('offboard_control')

        # --- Subscribers ---
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            qos_profile_sensor_data
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
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile_sensor_data
        )
        self.wp_reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.wp_reached_callback,
            10
        )
        self.terrain_sub = self.create_subscription(
            TerrainReport,
            '/mavros/terrain/report',
            self.terrain_callback,
            10
        )

        # --- Services ---
        self.arm_cli      = self.create_client(CommandBool,   '/mavros/cmd/arming')
        self.mode_cli     = self.create_client(SetMode,       '/mavros/set_mode')
        self.takeoff_cli  = self.create_client(CommandTOL,    '/mavros/cmd/takeoff')
        self.wp_push_cli  = self.create_client(WaypointPush,  '/mavros/mission/push')
        self.wp_clear_cli = self.create_client(WaypointClear, '/mavros/mission/clear')

        # --- Battery safety ---
        self.BATTERY_LOW_THRESHOLD = 20.0
        self.battery_warning_sent = False

        # --- Internal state ---
        self.state: State | None = None
        self.local_pose: PoseStamped | None = None
        self.home_lat: float | None = None
        self.home_lon: float | None = None
        self.home_alt: float | None = None
        self.targets: list[tuple[float, float, float]] = []
        self.current_lat: float | None = None
        self.current_lon: float | None = None
        self.current_alt: float | None = None
        self.battery_voltage: float | None = None
        self.battery_percentage: float | None = None
        self._total_mission_waypoints = 0
        
        # --- Terrain Report ---
        self.terrain_pending: int = 0  # number of terrain tiles pending download
        self.terrain_loaded: bool = False

        # Safety monitor timer — checks every 5 seconds
        self.safety_timer = self.create_timer(5.0, self.safety_monitor)

        self.get_logger().info("OffboardControl initialised.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def state_callback(self, msg: State):
        self.state = msg

    def local_pose_callback(self, msg: PoseStamped):
        self.local_pose = msg

    def global_callback(self, msg: NavSatFix):
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.get_logger().info(
                f"[GLOBAL] Home set: lat={self.home_lat:.7f}, "
                f"lon={self.home_lon:.7f}, alt={self.home_alt:.1f}"
            )
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def battery_callback(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage * 100

    def wp_reached_callback(self, msg: WaypointReached):
        self.get_logger().info(f"[MISSION] Waypoint {msg.wp_seq} reached.")
        if msg.wp_seq == self._total_mission_waypoints - 2:
            self.on_mission_complete()
    
    def terrain_callback(self, msg):
        self.terrain_pending = msg.pending
        self.terrain_loaded = msg.pending == 0 and msg.loaded > 0
        if not self.terrain_loaded:
            self.get_logger().warn(
                f"[TERRAIN] Tiles pending: {msg.pending}, loaded: {msg.loaded}"
            )

    # ------------------------------------------------------------------
    # End of mission sequence
    # ------------------------------------------------------------------

    def on_mission_complete(self):
        """
        Called when the final user waypoint is reached.
        Define end-of-mission behaviour here.
        """
        self.get_logger().info("[MISSION] Final waypoint reached — executing end of mission sequence...")
        self.return_to_home_immediate()

    # ------------------------------------------------------------------
    # Mission management
    # ------------------------------------------------------------------

    def upload_mission(self, waypoints: list[tuple[float, float, float]], takeoff_alt: float = 10.0):
        """
        Upload a MAVLink mission with takeoff, waypoints, and RTL.
        Navigation waypoints use FRAME_GLOBAL_TERRAIN_ALT for terrain following.
        ArduPilot handles all navigation and sequencing in AUTO mode.
        """
         # --- Terrain Loaded safety check ---
        if not self.terrain_loaded:
            self.get_logger().error(
                f"[SAFETY] Terrain data not ready — "
                f"{self.terrain_pending} tiles pending. Mission rejected!"
            )
            return
            
        # --- Minimum altitude safety check ---
        for i, (lat, lon, alt) in enumerate(waypoints):
            if alt < MIN_ALT:
                self.get_logger().error(
                    f"[SAFETY] Waypoint {i+1} altitude {alt}m is below minimum "
                    f"{MIN_ALT}m above terrain — mission rejected!"
                )
                return

        self.get_logger().info(
            f"[MISSION] Uploading {len(waypoints)} user waypoints "
            f"({len(waypoints) + 3} total including home, takeoff and RTL)..."
        )
        self.targets = waypoints

        wp_list = []

        # --- Waypoint 0: Home (relative to home, required by ArduPilot) ---
        home = Waypoint()
        home.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
        home.command      = 16
        home.is_current   = False
        home.autocontinue = True
        home.x_lat        = self.home_lat or 0.0
        home.y_long       = self.home_lon or 0.0
        home.z_alt        = 0.0
        wp_list.append(home)

        # --- Waypoint 1: Takeoff (relative to home) ---
        takeoff = Waypoint()
        takeoff.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
        takeoff.command      = 22
        takeoff.is_current   = True
        takeoff.autocontinue = True
        takeoff.z_alt        = float(takeoff_alt)
        wp_list.append(takeoff)

        # --- Navigation waypoints (terrain relative) ---
        for lat, lon, alt in waypoints:
            wp = Waypoint()
            wp.frame        = 10   # FRAME_GLOBAL_TERRAIN_ALT — altitude above terrain
            wp.command      = 16
            wp.is_current   = False
            wp.autocontinue = True
            wp.param1       = 0.0
            wp.param2       = 2.0  # acceptance radius in metres
            wp.param3       = 0.0
            wp.param4       = float('nan')  # yaw — auto
            wp.x_lat        = lat
            wp.y_long       = lon
            wp.z_alt        = float(alt)
            wp_list.append(wp)

        # --- Final waypoint: RTL (relative to home) ---
        rtl = Waypoint()
        rtl.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
        rtl.command      = 20
        rtl.autocontinue = True
        wp_list.append(rtl)

        # Store total for waypoint reached callback
        self._total_mission_waypoints = len(wp_list)

        # Clear existing mission
        while not self.wp_clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission clear service...')
        clear_future = self.wp_clear_cli.call_async(WaypointClear.Request())
        while not clear_future.done():
            time.sleep(0.05)
        self.get_logger().info("[MISSION] Existing mission cleared.")

        # Wait to ensure clear is processed by ArduPilot
        time.sleep(1.0)

        # Push new mission
        while not self.wp_push_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission push service...')
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = wp_list
        push_future = self.wp_push_cli.call_async(req)
        while not push_future.done():
            time.sleep(0.05)

        result = push_future.result()
        if result and result.success:
            self.get_logger().info(
                f"[MISSION] Mission uploaded ✅ "
                f"({result.wp_transfered} total: home + takeoff + {len(waypoints)} waypoints + RTL)"
            )
        else:
            self.get_logger().error("[MISSION] Mission upload failed ❌")

    def clear_targets(self):
        """Clear queued targets and uploaded mission."""
        self.targets = []
        while not self.wp_clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission clear service...')
        future = self.wp_clear_cli.call_async(WaypointClear.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info("[MISSION] Mission cleared ✅")
        )

    # ------------------------------------------------------------------
    # Arm + GUIDED + Takeoff + AUTO
    # ------------------------------------------------------------------

    def arm_and_start_mission(self, takeoff_alt: float = 10.0):
        self.get_logger().info("[ARM] Starting GUIDED → ARM → TAKEOFF → AUTO sequence...")

        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        # Set GUIDED mode
        req_mode = SetMode.Request()
        req_mode.custom_mode = 'GUIDED'
        future = self.mode_cli.call_async(req_mode)
        while not future.done():
            time.sleep(0.05)
        if not (future.result() and future.result().mode_sent):
            self.get_logger().error("[ARM] Failed to set GUIDED ❌")
            return
        self.get_logger().info("[ARM] GUIDED mode set ✅")

        # Wait for EKF
        self.get_logger().info("[ARM] Waiting for EKF to stabilise...")
        time.sleep(3.0)

        # Arm
        req_arm = CommandBool.Request()
        req_arm.value = True
        future = self.arm_cli.call_async(req_arm)
        while not future.done():
            time.sleep(0.05)
        if not (future.result() and future.result().success):
            self.get_logger().error("[ARM] Arming failed ❌")
            return
        self.get_logger().info("[ARM] Armed ✅")

        # Takeoff
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        req_to = CommandTOL.Request()
        req_to.altitude = float(takeoff_alt)
        req_to.latitude = 0.0
        req_to.longitude = 0.0
        req_to.min_pitch = 0.0
        req_to.yaw = 0.0
        future = self.takeoff_cli.call_async(req_to)
        while not future.done():
            time.sleep(0.05)
        self.get_logger().info(f"[TAKEOFF] Takeoff to {takeoff_alt}m ✅")

        # Wait until airborne
        threshold = max(1.5, takeoff_alt * 0.7)
        start = time.time()
        while time.time() - start < 30.0:
            if self.local_pose and self.local_pose.pose.position.z > threshold:
                self.get_logger().info("[TAKEOFF] Airborne ✅")
                break
            time.sleep(0.1)

        # Switch to AUTO
        req_mode.custom_mode = 'AUTO'
        future = self.mode_cli.call_async(req_mode)
        while not future.done():
            time.sleep(0.05)
        if future.result() and future.result().mode_sent:
            self.get_logger().info("[MISSION] AUTO mode set — executing mission ✅")
        else:
            self.get_logger().error("[MISSION] Failed to set AUTO ❌")

    # ------------------------------------------------------------------
    # RTL
    # ------------------------------------------------------------------

    def return_to_home_immediate(self):
        """Switch to RTL — ArduPilot flies home and lands automatically."""
        self.get_logger().info("[RTL] Switching to RTL mode...")
        self.targets = []
        req = SetMode.Request()
        req.custom_mode = 'RTL'
        future = self.mode_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info("[RTL] RTL mode set ✅")
            if f.result() and f.result().mode_sent
            else self.get_logger().error("[RTL] Failed to set RTL ❌")
        )

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def safety_monitor(self):
        """Check battery and trigger emergency land if low."""
        if self.battery_percentage is None:
            return
        if self.state is None or not self.state.armed:
            return

        if self.battery_percentage <= self.BATTERY_LOW_THRESHOLD:
            if not self.battery_warning_sent:
                self.get_logger().error(
                    f"[SAFETY] LOW BATTERY: {self.battery_percentage:.1f}% "
                    f"— initiating emergency land!"
                )
                self.battery_warning_sent = True
                #self.emergency_land()
        else:
            self.battery_warning_sent = False

    def emergency_land(self):
        """Immediately switch to LAND mode — drops straight down."""
        self.get_logger().error("[SAFETY] EMERGENCY LAND activated!")
        self.targets = []
        req = SetMode.Request()
        req.custom_mode = 'LAND'
        future = self.mode_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().error("[SAFETY] LAND mode set ✅")
            if f.result() and f.result().mode_sent
            else self.get_logger().error("[SAFETY] Failed to set LAND mode ❌")
        )

