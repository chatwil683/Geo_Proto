import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class offboardControlMock(Node):
    """Mock version of OffboardControl for testing without a drone."""

    def __init__(self):
        super().__init__('offboard_control_mock')
        self.current_state = type("State", (), {})()  # simple object
        self.current_state.connected = True
        self.current_state.armed = True
        self.current_state.mode = "OFFBOARD"

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.z = 2.0

        self.target_list = []

    def set_target(self, x, y, z):
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.target_list.append((x, y, z))
        self.get_logger().info(f"[MOCK] Target set to ({x}, {y}, {z})")


    def arm_and_offboard(self):
        self.get_logger().info("[MOCK] Arm and OFFBOARD called")

