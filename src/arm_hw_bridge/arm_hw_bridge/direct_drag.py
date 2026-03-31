import math
import time

import rclpy
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


class DirectDragController(Node):
    def __init__(self):
        super().__init__('direct_drag_controller')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('use_serial', True)
        self.declare_parameter('serial_debug', False)
        self.declare_parameter('hardware_motion_enabled', False)
        self.declare_parameter('serial_status_poll_sec', 1.0)
        self.declare_parameter('steps_per_revolution', 6400.0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('marker_namespace', 'roboforge_drag_target')
        self.declare_parameter('marker_scale', 0.34)
        self.declare_parameter('joint_marker_parent_frames', ['Base_Link', 'Link_1', 'Link_2', 'Link_3'])
        self.declare_parameter('joint_marker_names', ['joint_0', 'joint_1', 'joint_2', 'joint_3'])
        self.declare_parameter('joint_marker_descriptions', ['Base J0', 'Shoulder J1', 'Elbow J2', 'Wrist J3'])
        self.declare_parameter(
            'joint_marker_offsets',
            [
                0.0, 0.084, 0.13,
                -0.0506, -0.111, 0.0,
                0.29517649, 0.2861745, -0.0098,
                0.32042091, 0.12098218, 0.058,
            ],
        )
        self.declare_parameter(
            'joint_control_orientations',
            [
                0.70710678, 0.0, 0.0, -0.70710678,
                0.70710678, 0.0, -0.70710678, 0.0,
                0.70710678, 0.0, -0.70710678, 0.0,
                0.70710678, 0.0, 0.0, -0.70710678,
            ],
        )
        self.declare_parameter('joint_drag_signs', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('joint_click_step_rad', [0.16, 0.10, 0.10, 0.16])
        self.declare_parameter('joint_state_rate_hz', 20.0)
        self.declare_parameter('command_rate_hz', 40.0)
        self.declare_parameter('joint_smoothing_alpha', 0.28)
        self.declare_parameter('joint_goal_tolerance', 0.01)
        self.declare_parameter('joint_velocity_limits', [0.55, 0.45, 0.45, 0.6])
        self.declare_parameter('joint_lower_limits', [-math.pi, -1.48353, -1.396263, -math.pi])
        self.declare_parameter('joint_upper_limits', [math.pi, 0.349066, 1.570796, math.pi])
        self.declare_parameter('home_positions_rad', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            'joint_names',
            [
                'Base_Link_Revolute-11',
                'Link_1_Revolute-12',
                'Link_2_Revolute-13',
                'Link_3_Revolute-8',
            ],
        )

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_namespace = self.get_parameter('marker_namespace').value
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.joint_marker_parent_frames = list(self.get_parameter('joint_marker_parent_frames').value)
        self.joint_marker_names = list(self.get_parameter('joint_marker_names').value)
        self.joint_marker_descriptions = list(self.get_parameter('joint_marker_descriptions').value)
        marker_offsets_flat = [float(v) for v in self.get_parameter('joint_marker_offsets').value]
        self.joint_marker_offsets = [marker_offsets_flat[i:i + 3] for i in range(0, len(marker_offsets_flat), 3)]
        control_orientations_flat = [float(v) for v in self.get_parameter('joint_control_orientations').value]
        self.joint_control_orientations = [control_orientations_flat[i:i + 4] for i in range(0, len(control_orientations_flat), 4)]
        self.joint_drag_signs = [float(v) for v in self.get_parameter('joint_drag_signs').value]
        self.joint_click_step_rad = [float(v) for v in self.get_parameter('joint_click_step_rad').value]
        self.steps_per_rad = float(self.get_parameter('steps_per_revolution').value) / (2.0 * math.pi)
        self.hardware_motion_enabled = bool(self.get_parameter('hardware_motion_enabled').value)
        self.serial_debug = bool(self.get_parameter('serial_debug').value)
        self.serial_status_poll_sec = max(0.0, float(self.get_parameter('serial_status_poll_sec').value))
        self.joint_smoothing_alpha = float(self.get_parameter('joint_smoothing_alpha').value)
        self.joint_goal_tolerance = float(self.get_parameter('joint_goal_tolerance').value)
        self.joint_velocity_limits = [float(v) for v in self.get_parameter('joint_velocity_limits').value]
        self.joint_lower_limits = [float(v) for v in self.get_parameter('joint_lower_limits').value]
        self.joint_upper_limits = [float(v) for v in self.get_parameter('joint_upper_limits').value]
        self.home_positions_rad = [float(v) for v in self.get_parameter('home_positions_rad').value]

        joint_count = len(self.joint_names)
        if len(self.joint_marker_parent_frames) != joint_count:
            self.joint_marker_parent_frames = ['Base_Link', 'Link_1', 'Link_2', 'Link_3'][:joint_count]
        if len(self.joint_marker_names) != joint_count:
            self.joint_marker_names = [f'joint_{index}' for index in range(joint_count)]
        if len(self.joint_marker_descriptions) != joint_count:
            self.joint_marker_descriptions = [f'Joint {index}' for index in range(joint_count)]
        if len(self.joint_marker_offsets) != joint_count:
            self.joint_marker_offsets = [[0.0, 0.0, 0.0] for _ in range(joint_count)]
        if len(self.joint_control_orientations) != joint_count:
            self.joint_control_orientations = [[1.0, 0.0, 0.0, 0.0] for _ in range(joint_count)]
        if len(self.joint_drag_signs) != joint_count:
            self.joint_drag_signs = [1.0] * joint_count
        if len(self.joint_click_step_rad) != joint_count:
            self.joint_click_step_rad = [0.1] * joint_count
        if len(self.joint_velocity_limits) != joint_count:
            self.joint_velocity_limits = [1.0] * joint_count
        if len(self.joint_lower_limits) != joint_count:
            self.joint_lower_limits = [-math.pi] * joint_count
        if len(self.joint_upper_limits) != joint_count:
            self.joint_upper_limits = [math.pi] * joint_count
        if len(self.home_positions_rad) != joint_count:
            self.home_positions_rad = [0.0] * joint_count

        self.current_positions = list(self.home_positions_rad)
        self.target_positions = list(self.home_positions_rad)
        self.last_sent_steps = None
        self.marker_initialized = False
        self.last_command_update_time = self.get_clock().now()
        self.last_missing_frames = set()
        self.last_status_poll_time = self.get_clock().now()
        self.serial_command_count = 0
        self.awaiting_move_ack = False

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_server = InteractiveMarkerServer(self, self.marker_namespace)

        joint_state_rate_hz = float(self.get_parameter('joint_state_rate_hz').value)
        command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.create_timer(1.0 / joint_state_rate_hz, self.publish_joint_state)
        self.create_timer(1.0 / command_rate_hz, self.update_command_motion)
        self.create_timer(0.5, self.initialize_markers)
        self.create_timer(0.5, self.refresh_marker_poses)
        self.create_timer(0.05, self.poll_serial_feedback)

        self.serial_conn = None
        self.configure_serial()
        self.publish_joint_state()
        self.get_logger().info('Direct drag controller ready in direct joint mode')

    def configure_serial(self):
        if not bool(self.get_parameter('use_serial').value):
            self.get_logger().info('Serial output disabled by parameter')
            return
        if serial is None:
            self.get_logger().error('pyserial is not installed, serial output is unavailable')
            return

        port = self.get_parameter('serial_port').value
        baud_rate = int(self.get_parameter('baud_rate').value)
        try:
            self.serial_conn = serial.Serial(port, baud_rate, timeout=0.2)
            time.sleep(2.0)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.write_serial_line('MODE ROS')
            reply = self.read_serial_line()
            if reply:
                self.log_serial_rx(reply)
            self.get_logger().info(f'Connected to hardware on {port}')
        except Exception as exc:  # pragma: no cover
            self.serial_conn = None
            self.get_logger().error(f'Failed to open serial port {port}: {exc}')
        if not self.hardware_motion_enabled:
            self.get_logger().warn('Hardware motion disabled; RViz-only mode is active')
        elif self.serial_debug:
            self.get_logger().info('Serial debug logging enabled')

    def on_joint_state(self, msg: JointState):
        name_to_position = dict(zip(msg.name, msg.position))
        if not all(name in name_to_position for name in self.joint_names):
            return
        self.current_positions = [name_to_position[name] for name in self.joint_names]

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.current_positions)
        self.joint_pub.publish(msg)

    def initialize_markers(self):
        if self.marker_initialized:
            return
        for index in range(len(self.joint_names)):
            self.create_joint_marker(index)
        self.marker_initialized = True
        self.marker_server.applyChanges()
        self.get_logger().info('RViz joint controls initialized on /roboforge_drag_target')

    def refresh_marker_poses(self):
        if not self.marker_initialized:
            return
        for index, marker_name in enumerate(self.joint_marker_names):
            self.marker_server.setPose(marker_name, self.anchor_pose(index))
        self.marker_server.applyChanges()

    def create_joint_marker(self, index):
        marker = InteractiveMarker()
        marker.header.frame_id = self.world_frame
        marker.name = self.joint_marker_names[index]
        marker.description = self.joint_marker_descriptions[index]
        marker.scale = self.marker_scale
        marker.pose = self.anchor_pose(index)

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.interaction_mode = InteractiveMarkerControl.NONE
        visual.markers.append(self.make_marker_visual(index))
        marker.controls.append(visual)

        orientation = self.joint_control_orientations[index]
        if index in (0, 3):
            marker.controls.append(
                self.make_ring_jog_control(
                    'jog_positive',
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    orientation[3],
                    index,
                    direction=1.0,
                )
            )
            marker.controls.append(
                self.make_ring_jog_control(
                    'jog_negative',
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    orientation[3],
                    index,
                    direction=-1.0,
                )
            )
        else:
            marker.controls.append(
                self.make_jog_button_control(
                    'jog_positive',
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    orientation[3],
                    index,
                    direction=1.0,
                )
            )
            marker.controls.append(
                self.make_jog_button_control(
                    'jog_negative',
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    orientation[3],
                    index,
                    direction=-1.0,
                )
            )
        self.marker_server.insert(
            marker,
            feedback_callback=lambda feedback, i=index: self.process_joint_feedback(i, feedback),
        )

    def process_joint_feedback(self, index, feedback):
        if feedback.event_type not in (feedback.BUTTON_CLICK, feedback.MOUSE_DOWN):
            return

        if feedback.control_name == 'jog_positive':
            direction = 1.0
        elif feedback.control_name == 'jog_negative':
            direction = -1.0
        else:
            return

        target_delta = direction * self.joint_drag_signs[index] * self.joint_click_step_rad[index]
        new_target = self.clamp_joint(index, self.target_positions[index] + target_delta)
        if abs(new_target - self.target_positions[index]) <= 1e-5:
            return
        self.target_positions[index] = new_target
        self.get_logger().info(
            f'Jog {self.joint_marker_descriptions[index]} {self.jog_label(index, direction)} -> {new_target:.3f} rad',
            throttle_duration_sec=0.2,
        )

    def reset_joint_marker(self, index):
        self.marker_server.setPose(self.joint_marker_names[index], self.anchor_pose(index))
        self.marker_server.applyChanges()

    def anchor_pose(self, index):
        pose = self.lookup_joint_pose(index)
        if pose is not None:
            return pose
        pose = Pose()
        pose.position.x = self.joint_marker_offsets[index][0]
        pose.position.y = self.joint_marker_offsets[index][1]
        pose.position.z = self.joint_marker_offsets[index][2]
        pose.orientation.w = 1.0
        return pose

    def lookup_joint_pose(self, index):
        frame = self.joint_marker_parent_frames[index]
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if frame not in self.last_missing_frames:
                self.last_missing_frames.add(frame)
                self.get_logger().warn(f'Using fallback marker pose for {frame}: {exc}', throttle_duration_sec=2.0)
            return None

        if frame in self.last_missing_frames:
            self.last_missing_frames.remove(frame)
            self.get_logger().info(f'TF available for {frame}; marker anchored from world frame')

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation.w = 1.0
        return pose

    def update_command_motion(self):
        now = self.get_clock().now()
        dt = (now - self.last_command_update_time).nanoseconds / 1e9
        self.last_command_update_time = now
        if dt <= 0.0:
            return

        moved = False
        min_joint_step = 0.5 / self.steps_per_rad
        for index, target in enumerate(self.target_positions):
            current = self.current_positions[index]
            error = target - current
            if abs(error) <= self.joint_goal_tolerance:
                continue
            max_step = self.joint_velocity_limits[index] * dt
            filtered = abs(error) * self.joint_smoothing_alpha
            step = min(max_step, max(min_joint_step, filtered))
            self.current_positions[index] += math.copysign(min(abs(error), step), error)
            moved = True
        if not moved:
            return
        self.publish_joint_state()
        if self.hardware_motion_enabled:
            self.send_serial_command()

    def send_serial_command(self):
        if not self.hardware_motion_enabled:
            return
        steps = [int(round(position * self.steps_per_rad)) for position in self.current_positions]
        if steps == self.last_sent_steps:
            return
        self.last_sent_steps = steps
        cmd = f'MOVEABS {steps[0]} {steps[1]} {steps[2]} {steps[3]}'
        if self.serial_conn is None:
            self.get_logger().info(cmd, throttle_duration_sec=1.0)
            return
        try:
            self.serial_command_count += 1
            self.awaiting_move_ack = True
            if self.serial_debug:
                self.get_logger().info(
                    f'MOVEABS #{self.serial_command_count} '
                    'rad=[' + ', '.join(f'{position:.3f}' for position in self.current_positions) +
                    '] target_rad=[' + ', '.join(f'{position:.3f}' for position in self.target_positions) +
                    '] steps=[' + ', '.join(str(step) for step in steps) + ']'
                )
            self.write_serial_line(cmd)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Serial write failed: {exc}')

    def write_serial_line(self, line: str):
        if self.serial_conn is None:
            return
        payload = f'{line}\n'.encode()
        self.log_serial_tx(line)
        self.serial_conn.write(payload)
        self.serial_conn.flush()

    def read_serial_line(self):
        if self.serial_conn is None:
            return ''
        return self.serial_conn.readline().decode(errors='ignore').strip()

    def poll_serial_feedback(self):
        if self.serial_conn is None:
            return
        try:
            self.maybe_request_status()
            while self.serial_conn.in_waiting > 0:
                reply = self.read_serial_line()
                if not reply:
                    continue
                self.log_serial_rx(reply)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Serial read failed: {exc}')

    def log_serial_tx(self, line: str):
        if self.serial_debug:
            self.get_logger().info(f'Serial TX: {line}')

    def log_serial_rx(self, line: str):
        if line == 'OK MOVEABS':
            self.awaiting_move_ack = False
        if self.serial_debug:
            self.get_logger().info(f'Serial RX: {line}')
        elif line != 'OK MOVEABS':
            self.get_logger().info(f'Arduino: {line}')

    def maybe_request_status(self):
        if not self.hardware_motion_enabled or self.serial_status_poll_sec <= 0.0:
            return
        now = self.get_clock().now()
        elapsed = (now - self.last_status_poll_time).nanoseconds / 1e9
        if elapsed < self.serial_status_poll_sec:
            return
        self.last_status_poll_time = now
        if self.serial_debug and self.awaiting_move_ack:
            self.get_logger().info('Still waiting for OK MOVEABS; polling STATUS')
        self.write_serial_line('STATUS')

    def clamp_joint(self, index, angle):
        return max(self.joint_lower_limits[index], min(self.joint_upper_limits[index], angle))

    def make_marker_visual(self, index):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self.marker_scale * 0.22
        marker.scale.y = self.marker_scale * 0.22
        marker.scale.z = self.marker_scale * 0.22
        r, g, b = self.joint_color(index)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.95
        return marker

    def make_jog_button_control(self, name, w, x, y, z, index, direction):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.extend(self.make_jog_button_markers(index, direction))
        return control

    def make_jog_button_markers(self, index, direction):
        body = Marker()
        body.type = Marker.ARROW
        body.scale.x = self.marker_scale * 0.75
        body.scale.y = self.marker_scale * 0.22
        body.scale.z = self.marker_scale * 0.22
        body.pose.position.y = direction * self.marker_scale * 0.42
        if direction < 0.0:
            body.pose.orientation.z = 1.0
            body.pose.orientation.w = 0.0

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.scale.z = self.marker_scale * 0.20
        label.pose.position.y = direction * self.marker_scale * 0.70
        label.pose.position.z = self.marker_scale * 0.14
        label.text = self.jog_label(index, direction)

        r, g, b = self.joint_color(index)
        for marker in (body, label):
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.98
        return [body, label]

    def make_ring_jog_control(self, name, w, x, y, z, index, direction):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.extend(self.make_ring_jog_markers(index, direction))
        return control

    def make_ring_jog_markers(self, index, direction):
        arc = Marker()
        arc.type = Marker.CYLINDER
        arc.scale.x = self.marker_scale * 1.05
        arc.scale.y = self.marker_scale * 1.05
        arc.scale.z = self.marker_scale * 0.055
        arc.pose.orientation.y = math.sqrt(0.5)
        arc.pose.orientation.w = math.sqrt(0.5)
        arc.pose.position.y = direction * self.marker_scale * 0.32

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = self.marker_scale * 0.45
        arrow.scale.y = self.marker_scale * 0.16
        arrow.scale.z = self.marker_scale * 0.16
        arrow.pose.position.y = direction * self.marker_scale * 0.62
        if direction < 0.0:
            arrow.pose.orientation.z = 1.0
            arrow.pose.orientation.w = 0.0

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.scale.z = self.marker_scale * 0.18
        label.pose.position.y = direction * self.marker_scale * 0.82
        label.pose.position.z = self.marker_scale * 0.12
        label.text = self.jog_label(index, direction)

        r, g, b = self.joint_color(index)
        arc.color.r = r
        arc.color.g = g
        arc.color.b = b
        arc.color.a = 0.28
        for marker in (arrow, label):
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.98
        return [arc, arrow, label]

    def jog_label(self, index, direction):
        labels = [
            ('CCW', 'CW'),
            ('DOWN', 'UP'),
            ('DOWN', 'UP'),
            ('CCW', 'CW'),
        ]
        negative_label, positive_label = labels[index % len(labels)]
        return positive_label if direction > 0.0 else negative_label

    def joint_color(self, index):
        colors = [
            (0.95, 0.45, 0.12),
            (0.12, 0.72, 0.95),
            (0.18, 0.82, 0.36),
            (0.95, 0.82, 0.18),
        ]
        return colors[index % len(colors)]



def main():
    rclpy.init()
    node = DirectDragController()
    try:
        rclpy.spin(node)
    finally:
        if node.serial_conn is not None:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()
