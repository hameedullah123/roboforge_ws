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
SENSITIVITY = 1.0
MAX_STEP = 50
MIN_DX = 2


class MouseBaseController:
    def __init__(self):
        if serial is None:
            raise RuntimeError('pyserial is not installed')

        self.serial_conn = serial.Serial(PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(STARTUP_DELAY_SEC)
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()
        self.serial_conn.write(b'MODE ROS\n')
        self.serial_conn.flush()

        self.base_pos = 0
        self.last_x = None
        self.listener = None

    def on_move(self, x, y):
        if self.last_x is None:
            self.last_x = x
            return

        dx = x - self.last_x
        self.last_x = x

        if abs(dx) < MIN_DX:
            return

        delta = int(dx * SENSITIVITY)
        if delta > MAX_STEP:
            delta = MAX_STEP
        elif delta < -MAX_STEP:
            delta = -MAX_STEP

        if delta == 0:
            return

        self.base_pos += delta
        self.send_moveabs()
        print(f'BASE: {self.base_pos}', flush=True)

    def on_click(self, x, y, button, pressed):
        if pressed and button == mouse.Button.right:
            self.base_pos = 0
            self.send_moveabs()
            print('RESET', flush=True)

    def send_moveabs(self):
        cmd = f'MOVEABS {self.base_pos} 0 0 0\n'
        self.serial_conn.write(cmd.encode())
        self.serial_conn.flush()

    def run(self):
        print('Mouse control started (BASE)', flush=True)
        print('Move mouse right = base clockwise, left = base counterclockwise', flush=True)
        print('Right click resets base to zero', flush=True)
        self.listener = mouse.Listener(on_move=self.on_move, on_click=self.on_click)
        self.listener.start()
        self.listener.join()

    def close(self):
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
        controller = MouseBaseController()
        controller.run()
    except KeyboardInterrupt:
        print('Mouse control stopped', flush=True)
    except Exception as exc:
        print(f'Failed to start mouse control: {exc}', file=sys.stderr, flush=True)
        sys.exit(1)
    finally:
        if controller is not None:
            controller.close()


if __name__ == '__main__':
    main()
