import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import serial
import time
import math

STEPS_PER_RAD = 6400 / (2 * math.pi)

class ArmBridge(Node):

    def __init__(self):
        super().__init__('arm_bridge')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)

        self.get_logger().info("Connected to Arduino")

        self.send_cmd("MODE ROS")

        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10
        )

    def send_cmd(self, cmd):
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.1)
        response = self.ser.read_all().decode(errors='ignore')
        print(response)

    def trajectory_callback(self, msg):
        print("TRAJECTORY RECEIVED")  # debug

        if len(msg.trajectory) == 0:
            return

        traj = msg.trajectory[0].joint_trajectory

        if len(traj.points) == 0:
            return

        point = traj.points[-1]

        steps = []
        for angle in point.positions:
            s = int(angle * STEPS_PER_RAD)
            steps.append(s)

        while len(steps) < 4:
            steps.append(0)

        cmd = f"MOVEABS {steps[0]} {steps[1]} {steps[2]} {steps[3]}"
        self.get_logger().info(f"Sending: {cmd}")

        self.send_cmd(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArmBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
