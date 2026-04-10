import threading

import rclpy
from geo_proto.offboardControl import OffboardControl
from geo_proto.webServer import start_server


def main():
    rclpy.init()
    node = OffboardControl()
    node.get_logger().info("[MAIN] OffboardControl node initialised.")

    # Start FastAPI server in a background thread
    threading.Thread(
        target=start_server,
        args=(node,),
        daemon=True
    ).start()

    node.get_logger().info("[MAIN] FastAPI server started on port 8000.")

    # Spin ROS forever
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[MAIN] KeyboardInterrupt → shutting down")
    finally:
        # Stop LED thread and cleanup GPIO first
        #node.led.cleanup()
        # Then destroy ROS node
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("[MAIN] ROS shutdown complete.")


if __name__ == "__main__":
    main()

