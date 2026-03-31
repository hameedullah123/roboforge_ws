import math
import signal
import sys
import time

from pynput import mouse

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.05
STARTUP_DELAY_SEC = 2.0
POLL_INTERVAL_SEC = 0.02
HEARTBEAT_SEC = 0.10
STATUS_POLL_SEC = 1.0
IDLE_STOP_SEC = 0.12
MOVEABS_UPDATE_SEC = 0.10

MIN_DX_PX = 1
MAX_DX_PX = 48
MIN_DY_PX = 1
MAX_DY_PX = 48
MAX_SPEED_LEVEL = 6

BASE_STEP_RATE_PER_LEVEL = 180.0
VELOCITY_RAMP_STEP = 2
FIXED_SHOULDER_STEP_CHUNK = 500
SHOULDER_SETTLE_TOL_STEPS = 4
SHOULDER_TRIGGER_PX = 8
SHOULDER_MAX_CHUNKS_PER_EVENT = 2
FIXED_ELBOW_STEP_CHUNK = 500
ELBOW_SETTLE_TOL_STEPS = 2
ELBOW_TRIGGER_PX = 6
ELBOW_MAX_CHUNKS_PER_EVENT = 1
FIXED_WRIST_STEP_CHUNK = 240
WRIST_SETTLE_TOL_STEPS = 4

HOME_STEPS = [0, 0, 0, 0]
JOINT_LOWER_LIMITS_RAD = [-math.pi, -1.48353, -math.pi, -math.pi]
JOINT_UPPER_LIMITS_RAD = [math.pi, 0.349066, math.pi, math.pi]
STEPS_PER_RAD = 6400.0 / (2.0 * math.pi)

# Adjust signs here if a joint moves opposite to the intended mouse direction.
BASE_SIGN = 1
SHOULDER_SIGN = 1
ELBOW_SIGN = -1
WRIST_SIGN = 1


class MouseArmDirectController:
    def __init__(self):
        if serial is None:
            raise RuntimeError('pyserial is not installed')

        self.serial_conn = serial.Serial(PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(STARTUP_DELAY_SEC)
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        self.last_x = None
        self.last_y = None
        self.base_target_velocity = 0
        self.base_sent_velocity = 0
        self.last_send_time = 0.0
        self.last_status_time = 0.0
        self.last_motion_time = 0.0
        self.last_moveabs_time = time.monotonic()
        self.current_steps = list(HOME_STEPS)
        self.commanded_steps = list(HOME_STEPS)
        self.feedback_current_steps = list(HOME_STEPS)
        self.feedback_target_steps = list(HOME_STEPS)
        self.have_feedback = False
        self.last_sent_steps = None
        self.listener = None
        self.running = True

        self.joint_lower_steps = [int(round(limit * STEPS_PER_RAD)) for limit in JOINT_LOWER_LIMITS_RAD]
        self.joint_upper_steps = [int(round(limit * STEPS_PER_RAD)) for limit in JOINT_UPPER_LIMITS_RAD]

        self.send_raw_command('MODE ROS', force=True)
        time.sleep(0.2)
        self.poll_serial_feedback(time.monotonic())

    def on_move(self, x, y):
        now = time.monotonic()
        if self.last_x is None or self.last_y is None:
            self.last_x = x
            self.last_y = y
            self.last_motion_time = now
            return

        dx = x - self.last_x
        dy = y - self.last_y
        self.last_x = x
        self.last_y = y

        self.base_target_velocity = self.axis_to_velocity(dx, MIN_DX_PX, MAX_DX_PX)
        self.try_queue_lift_jog(-dy)
        self.last_motion_time = now

    def on_click(self, x, y, button, pressed):
        if not pressed:
            return
        if button == mouse.Button.right:
            self.base_target_velocity = 0
            self.base_sent_velocity = 0
            self.last_x = None
            self.last_y = None
            self.send_current_steps(force=True)
            print('STOP', flush=True)
        elif button == mouse.Button.middle:
            self.base_target_velocity = 0
            self.base_sent_velocity = 0
            self.last_x = None
            self.last_y = None
            self.send_raw_command(
                f'HOME {HOME_STEPS[0]} {HOME_STEPS[1]} {HOME_STEPS[2]} {HOME_STEPS[3]}',
                force=True,
            )
            self.current_steps = list(HOME_STEPS)
            self.commanded_steps = list(HOME_STEPS)
            self.feedback_current_steps = list(HOME_STEPS)
            self.feedback_target_steps = list(HOME_STEPS)
            self.have_feedback = False
            self.last_sent_steps = None
            print('HOME', flush=True)

    def on_scroll(self, x, y, dx, dy):
        self.try_queue_wrist_jog(dy)

    def axis_to_velocity(self, delta: float, min_px: float, max_px: float) -> int:
        if abs(delta) < min_px:
            return 0
        magnitude = min(max_px, abs(delta))
        span = max(1.0, (max_px - min_px))
        scaled = 1.0 + ((magnitude - min_px) * (MAX_SPEED_LEVEL - 1) / span)
        velocity = min(MAX_SPEED_LEVEL, int(round(scaled)))
        return velocity if delta > 0 else -velocity

    def ramp_velocity(self, current: int, target: int) -> int:
        if current == target:
            return current
        if current < target:
            return min(target, current + VELOCITY_RAMP_STEP)
        return max(target, current - VELOCITY_RAMP_STEP)

    def clamp_step(self, index: int, step_value: int) -> int:
        return max(self.joint_lower_steps[index], min(self.joint_upper_steps[index], step_value))

    def try_queue_lift_jog(self, lift_input: float):
        if abs(lift_input) < min(SHOULDER_TRIGGER_PX, ELBOW_TRIGGER_PX):
            return
        if not self.shoulder_ready_for_next_chunk() or not self.elbow_ready_for_next_chunk():
            return
        shoulder_chunk_count = min(
            SHOULDER_MAX_CHUNKS_PER_EVENT,
            max(1, int(abs(lift_input) / SHOULDER_TRIGGER_PX)),
        )
        elbow_chunk_count = min(
            ELBOW_MAX_CHUNKS_PER_EVENT,
            max(1, int(abs(lift_input) / ELBOW_TRIGGER_PX)),
        )
        direction = 1 if lift_input > 0 else -1
        shoulder_delta = direction * SHOULDER_SIGN * FIXED_SHOULDER_STEP_CHUNK * shoulder_chunk_count
        elbow_delta = direction * ELBOW_SIGN * FIXED_ELBOW_STEP_CHUNK * elbow_chunk_count
        next_shoulder = self.clamp_step(1, self.current_steps[1] + shoulder_delta)
        next_elbow = self.clamp_step(2, self.current_steps[2] + elbow_delta)
        if next_shoulder == self.current_steps[1] and next_elbow == self.current_steps[2]:
            return
        self.current_steps[1] = next_shoulder
        self.current_steps[2] = next_elbow
        self.commanded_steps = list(self.current_steps)
        self.send_current_steps()

    def try_queue_wrist_jog(self, scroll_delta: float):
        if scroll_delta == 0:
            return
        if not self.wrist_ready_for_next_chunk():
            return
        direction = 1 if scroll_delta > 0 else -1
        wrist_delta = direction * WRIST_SIGN * FIXED_WRIST_STEP_CHUNK
        next_wrist = self.clamp_step(3, self.current_steps[3] + wrist_delta)
        if next_wrist == self.current_steps[3]:
            return
        self.current_steps[3] = next_wrist
        self.commanded_steps = list(self.current_steps)
        self.send_current_steps()

    def shoulder_ready_for_next_chunk(self) -> bool:
        if not self.have_feedback:
            return True
        return abs(self.feedback_current_steps[1] - self.commanded_steps[1]) <= SHOULDER_SETTLE_TOL_STEPS

    def elbow_ready_for_next_chunk(self) -> bool:
        if not self.have_feedback:
            return True
        return abs(self.feedback_current_steps[2] - self.commanded_steps[2]) <= ELBOW_SETTLE_TOL_STEPS

    def wrist_ready_for_next_chunk(self) -> bool:
        if not self.have_feedback:
            return True
        return abs(self.feedback_current_steps[3] - self.commanded_steps[3]) <= WRIST_SETTLE_TOL_STEPS

    def integrate_base_motion(self, now: float):
        dt = now - self.last_moveabs_time
        if dt < MOVEABS_UPDATE_SEC:
            return
        self.last_moveabs_time = now

        if self.base_sent_velocity == 0:
            return

        updated_steps = list(self.current_steps)
        base_delta = int(round(self.base_sent_velocity * BASE_SIGN * BASE_STEP_RATE_PER_LEVEL * dt))
        if base_delta != 0:
            updated_steps[0] = self.clamp_step(0, updated_steps[0] + base_delta)

        if updated_steps != self.current_steps:
            self.current_steps = updated_steps
            self.commanded_steps = list(updated_steps)
            self.send_current_steps()

    def send_raw_command(self, command: str, force: bool = False):
        now = time.monotonic()
        if not force and (now - self.last_send_time) < HEARTBEAT_SEC:
            return
        self.serial_conn.write((command + '\n').encode())
        self.serial_conn.flush()
        self.last_send_time = now
        print(f'TX: {command}', flush=True)

    def send_current_steps(self, force: bool = False):
        if not force and self.commanded_steps == self.last_sent_steps:
            return
        self.last_sent_steps = list(self.commanded_steps)
        self.send_raw_command(
            f'MOVEABS {self.commanded_steps[0]} {self.commanded_steps[1]} {self.commanded_steps[2]} {self.commanded_steps[3]}',
            force=force,
        )

    def poll_control_loop(self):
        while self.running:
            now = time.monotonic()
            if self.base_target_velocity != 0 and (now - self.last_motion_time) >= IDLE_STOP_SEC:
                self.base_target_velocity = 0

            self.base_sent_velocity = self.ramp_velocity(self.base_sent_velocity, self.base_target_velocity)
            self.integrate_base_motion(now)
            self.poll_serial_feedback(now)
            time.sleep(POLL_INTERVAL_SEC)

    def poll_serial_feedback(self, now: float):
        if (now - self.last_status_time) >= STATUS_POLL_SEC:
            self.last_status_time = now
            self.serial_conn.write(b'STATUS\n')
            self.serial_conn.flush()
        while self.serial_conn.in_waiting > 0:
            line = self.serial_conn.readline().decode(errors='ignore').strip()
            if line:
                self.parse_status_line(line)
                print(f'RX: {line}', flush=True)

    def parse_status_line(self, line: str):
        if not line.startswith('MODE=') or ' CUR=' not in line or ' TAR=' not in line:
            return
        try:
            after_cur = line.split(' CUR=', 1)[1]
            current_text, target_text = after_cur.split(' TAR=', 1)
            current_values = [int(part) for part in current_text.split(',')]
            target_values = [int(part) for part in target_text.split(',')]
        except (IndexError, ValueError):
            return
        if len(current_values) != 4 or len(target_values) != 4:
            return
        self.feedback_current_steps = current_values
        self.feedback_target_steps = target_values
        self.have_feedback = True

    def run(self):
        print('Mouse arm direct control started', flush=True)
        print('Move mouse left/right for base motion', flush=True)
        print('Move mouse up/down for combined shoulder + elbow jog steps', flush=True)
        print('Use scroll for wrist/gripper jog steps', flush=True)
        print('Right click = STOP, middle click = HOME', flush=True)

        self.listener = mouse.Listener(
            on_move=self.on_move,
            on_click=self.on_click,
            on_scroll=self.on_scroll,
        )
        self.listener.start()
        self.poll_control_loop()

    def close(self):
        self.running = False
        try:
            self.send_current_steps(force=True)
        except Exception:
            pass
        if self.listener is not None:
            self.listener.stop()
        if self.serial_conn is not None:
            self.serial_conn.close()


def main():
    controller = None

    def handle_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        controller = MouseArmDirectController()
        controller.run()
    except KeyboardInterrupt:
        print('Mouse arm direct control stopped', flush=True)
    except Exception as exc:
        print(f'Failed to start mouse arm direct control: {exc}', file=sys.stderr, flush=True)
        sys.exit(1)
    finally:
        if controller is not None:
            controller.close()


if __name__ == '__main__':
    main()
