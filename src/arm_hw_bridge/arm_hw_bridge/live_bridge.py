import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math


class LiveBridge(Node):

    def __init__(self):
        super().__init__('live_bridge')

        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.01)

        time.sleep(3)  # 🔥 allow Arduino reset

        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write(b"MODE ROS\n")
        time.sleep(0.5)

        self.get_logger().info("Connected to Arduino")
        self.get_logger().info("Switched to ROS MODE")

        self.latest_steps = None
        self.last_sent_steps = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.send_command)  # 10Hz

    def joint_callback(self, msg):

        name_to_pos = dict(zip(msg.name, msg.position))

        ordered = [
            name_to_pos.get("Base_Link_Revolute-11", 0.0),
            name_to_pos.get("Link_1_Revolute-12", 0.0),
            name_to_pos.get("Link_2_Revolute-13", 0.0),
            name_to_pos.get("Link_3_Revolute-8", 0.0),
        ]

        steps = [int(p * (6400 / (2 * math.pi))) for p in ordered]

        self.latest_steps = steps

    def send_command(self):

        if self.latest_steps is None:
            return

        if self.latest_steps == self.last_sent_steps:
            return

        cmd = f"MOVEABS {self.latest_steps[0]} {self.latest_steps[1]} {self.latest_steps[2]} {self.latest_steps[3]}\n"

        try:
            self.ser.write(cmd.encode())
            self.ser.flush()

            print("LIVE:", cmd.strip())

        except Exception as e:
            print("Serial error:", e)

        self.last_sent_steps = self.latest_steps


def main():
    rclpy.init()
    node = LiveBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
