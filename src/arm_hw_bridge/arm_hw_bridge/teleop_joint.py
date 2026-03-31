import serial
import time
import sys
import termios
import tty
import select

# 🔌 Serial
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.01)
time.sleep(3)

ser.write(b"MODE ROS\n")
time.sleep(0.5)

print("Connected to Arduino (ROS MODE)")

# Joint steps
joints = [0, 0, 0, 0]
STEP = 100

def get_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

# raw mode
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    while True:

        key = get_key()

        if key == 'q':
            break

        moved = False

        if key == 'a':
            joints[0] += STEP
            moved = True
        elif key == 'z':
            joints[0] -= STEP
            moved = True

        elif key == 's':
            joints[1] += STEP
            moved = True
        elif key == 'x':
            joints[1] -= STEP
            moved = True

        elif key == 'd':
            joints[2] += STEP
            moved = True
        elif key == 'c':
            joints[2] -= STEP
            moved = True

        elif key == 'f':
            joints[3] += STEP
            moved = True
        elif key == 'v':
            joints[3] -= STEP
            moved = True

        # ✅ Only send when key pressed
        if moved:
            cmd = f"MOVEABS {joints[0]} {joints[1]} {joints[2]} {joints[3]}\n"
            ser.write(cmd.encode())
            ser.flush()

            print("CMD:", cmd.strip())

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    ser.close()
