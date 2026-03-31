import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import serial
import time


class DragBridge(Node):

    def __init__(self):
        super().__init__('drag_bridge')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
        time.sleep(2)

        self.ser.write(b"MODE ROS\n")
        time.sleep(0.5)

        self.get_logger().info("RVIZ DRAG REAL CONTROL READY")

        self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.callback,
            10
        )

    def callback(self, msg):

        if not msg.trajectory:
            return

        traj = msg.trajectory[0]

        if not traj.joint_trajectory.points:
            return

        # 🔥 get latest point
        point = traj.joint_trajectory.points[-1]

        positions = point.positions

        # radians → steps
        steps = [int(p * (6400 / (2 * 3.14159))) for p in positions]

        cmd = f"MOVEABS {steps[0]} {steps[1]} {steps[2]} {steps[3]}\n"

        self.ser.write(cmd.encode())
        self.ser.flush()

        self.get_logger().info(f"SEND: {cmd.strip()}")


def main():
    rclpy.init()
    node = DragBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
