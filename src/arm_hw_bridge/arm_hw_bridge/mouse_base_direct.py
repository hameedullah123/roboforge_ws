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
MIN_DX_PX = 1
MAX_DX_PX = 48
MAX_SPEED_LEVEL = 6
POLL_INTERVAL_SEC = 0.02
HEARTBEAT_SEC = 0.15
STATUS_POLL_SEC = 1.0
VELOCITY_RAMP_STEP = 2
IDLE_STOP_SEC = 0.12


class MouseBaseDirectController:
    def __init__(self):
        if serial is None:
            raise RuntimeError('pyserial is not installed')

        self.serial_conn = serial.Serial(PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(STARTUP_DELAY_SEC)
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        self.last_x = None
        self.target_velocity = 0
        self.sent_velocity = 0
        self.last_send_time = 0.0
        self.last_status_time = 0.0
        self.last_motion_time = 0.0
        self.listener = None
        self.running = True

    def on_move(self, x, y):
        now = time.monotonic()
        if self.last_x is None:
            self.last_x = x
            self.last_motion_time = now
            return

        dx = x - self.last_x
        self.last_x = x

        velocity = self.dx_to_velocity(dx)
        self.last_motion_time = now
        self.target_velocity = velocity

    def dx_to_velocity(self, dx: float) -> int:
        if abs(dx) < MIN_DX_PX:
            return 0
        magnitude = min(MAX_DX_PX, abs(dx))
        span = max(1, (MAX_DX_PX - MIN_DX_PX))
        scaled = 1.0 + ((magnitude - MIN_DX_PX) * (MAX_SPEED_LEVEL - 1) / span)
        velocity = min(MAX_SPEED_LEVEL, int(round(scaled)))
        return velocity if dx > 0 else -velocity

    def on_click(self, x, y, button, pressed):
        if not pressed:
            return
        if button == mouse.Button.right:
            self.target_velocity = 0
            self.sent_velocity = 0
            self.last_x = None
            self.send_command('STOP', force=True)
            print('STOP', flush=True)
        elif button == mouse.Button.middle:
            self.target_velocity = 0
            self.sent_velocity = 0
            self.last_x = None
            self.send_command('HOME', force=True)
            print('HOME', flush=True)

    def ramp_velocity(self, current: int, target: int) -> int:
        if current == target:
            return current
        if current < target:
            return min(target, current + VELOCITY_RAMP_STEP)
        return max(target, current - VELOCITY_RAMP_STEP)

    def send_command(self, command: str, force: bool = False):
        now = time.monotonic()
        if not force and command == f'VEL {self.sent_velocity}' and (now - self.last_send_time) < HEARTBEAT_SEC:
            return
        self.serial_conn.write((command + "\n").encode())
        self.serial_conn.flush()
        self.last_send_time = now
        print(f'TX: {command}', flush=True)

    def send_velocity(self, velocity: int, force: bool = False):
        self.sent_velocity = velocity
        self.send_command(f'VEL {velocity}', force=force)

    def poll_control_loop(self):
        while self.running:
            now = time.monotonic()
            if self.target_velocity != 0 and (now - self.last_motion_time) >= IDLE_STOP_SEC:
                self.target_velocity = 0
            next_velocity = self.ramp_velocity(self.sent_velocity, self.target_velocity)
            heartbeat_due = (now - self.last_send_time) >= HEARTBEAT_SEC
            force_send = heartbeat_due or (next_velocity != self.sent_velocity)
            self.send_velocity(next_velocity, force=force_send)
            self.poll_serial_feedback(now)
            time.sleep(POLL_INTERVAL_SEC)

    def poll_serial_feedback(self, now: float):
        if (now - self.last_status_time) >= STATUS_POLL_SEC:
            self.last_status_time = now
            try:
                self.serial_conn.write(b"STATUS\n")
                self.serial_conn.flush()
            except Exception:
                pass
        while self.serial_conn.in_waiting > 0:
            line = self.serial_conn.readline().decode(errors='ignore').strip()
            if line:
                print(f'RX: {line}', flush=True)

    def run(self):
        print('Mouse base direct control started', flush=True)
        print('Move mouse right/left to send smooth signed velocity levels', flush=True)
        print('Right click = STOP, middle click = HOME', flush=True)

        self.listener = mouse.Listener(on_move=self.on_move, on_click=self.on_click)
        self.listener.start()
        self.poll_control_loop()

    def close(self):
        self.running = False
        self.target_velocity = 0
        try:
            self.send_command('STOP', force=True)
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
        controller = MouseBaseDirectController()
        controller.run()
    except KeyboardInterrupt:
        print('Mouse base direct control stopped', flush=True)
    except Exception as exc:
        print(f'Failed to start mouse base direct control: {exc}', file=sys.stderr, flush=True)
        sys.exit(1)
    finally:
        if controller is not None:
            controller.close()


if __name__ == '__main__':
    main()
