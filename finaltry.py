import serial
import argparse
import threading
from dronekit import connect
import json
import time
from rplidar import RPLidar

# Global variables to store the latest sensor data
latest_servo1_value = None
latest_servo3_value = None
latest_scan = None
MIN_DISTANCE = 500  # mm, obstacle distance threshold

def read_ddsm_serial(ddsm_ser):
    """Thread function to read and optionally log DDSM serial data."""
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass  # Optionally uncomment to print: print(f"[DDSM] {data}", end='')
        except Exception as e:
            print(f"[Error reading DDSM] {e}")

def scale_servo_to_speed(servo_value):
    """Maps servo PWM range to speed values, with None handling."""
    if servo_value is None:
        return 0
    # Maps servo_value (1000-2000 µs) to -100 to 100, centered around 1735
    return int((servo_value - 1500) / 500 * 100)

def process_scan(scan):
    """Process LiDAR scan to determine if front, left, or right sectors are clear."""
    front_clear = True
    left_clear = True
    right_clear = True
    print("reading lidar")
    for (_, angle, distance) in scan:
        if distance == 0:
            print("Distance 0")
            continue
        # Front sector: -10° to 10° (350° to 10°)
        if (angle >= 350 or angle <= 10) and distance < MIN_DISTANCE:
            print("Front Not Clear")
            front_clear = False
        # Right sector: 60° to 120°
        elif 60 <= angle <= 120 and distance < MIN_DISTANCE:
            print("Right Not Clear")
            right_clear = False
        # Left sector: 240° to 300°
        elif 240 <= angle <= 300 and distance < MIN_DISTANCE:
            print("Left Not Clear")
            left_clear = False
    return front_clear, left_clear, right_clear

def lidar_thread(lidar):
    """Thread function to continuously update the latest LiDAR scan."""
    global latest_scan
    try:
        for scan in lidar.iter_scans():
            print("lidar thread reading")
            latest_scan = scan
    except Exception as e:
        print(f"[LiDAR Error] {e}")

def main():
    # Parse command-line arguments for port configuration
    parser = argparse.ArgumentParser(description='Integrated Rover Control with LiDAR Obstacle Avoidance')
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='Pixhawk MAVLink port (e.g., /dev/ttyAMA0)')
    parser.add_argument('--lidar', type=str, default='/dev/ttyUSB0', help='LiDAR port (e.g., /dev/ttyUSB0)')
    args = parser.parse_args()

    # Connect to DDSM serial
    ddsm_ser = serial.Serial(args.ddsm_port, baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)

    # Start thread to read from DDSM
    threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # Connect to Pixhawk
    print("[Info] Connecting to Pixhawk...")
    vehicle = connect(args.pixhawk, wait_ready=True, baud=57600)
    print("[Info] Connected to Pixhawk.")

    # Define the servo output listener
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw  # Right-side wheels
        latest_servo3_value = message.servo3_raw  # Left-side wheels
        print(f"[Servo Output] Channel 1: {latest_servo1_value} µs, Channel 3: {latest_servo3_value} µs")

    # Connect to LiDAR
    lidar = RPLidar(args.lidar)
    threading.Thread(target=lidar_thread, args=(lidar,), daemon=True).start()

    try:
        while True:
            # Retrieve latest servo values
            servo1 = latest_servo1_value
            servo3 = latest_servo3_value

            # Convert servo values to wheel speeds (scaled to -100 to 100 range)
            speed_right = scale_servo_to_speed(servo1)
            speed_left = scale_servo_to_speed(servo3)
            speed_right = max(-100, min(100, speed_right))  # Limit speed to range -100 to 100
            speed_left = max(-100, min(100, speed_left))    # Limit speed to range -100 to 100

            # Obstacle avoidance logic when moving forward
            if speed_left > 0 and speed_right > 0 and latest_scan is not None:
                front_clear, left_clear, right_clear = process_scan(latest_scan)
                if not front_clear:
                    print("[Obstacle Detected] Stopping and taking action...")
                    if left_clear:
                        print("[Action] Turning right")
                        # Turn right
                        speed_left = -50
                        speed_right = 50
                    elif right_clear:
                        print("[Action] Turning left")
                        # Turn left
                        speed_left = 50
                        speed_right = -50
                    else:
                        print("[Action] Stopping - No path clear")
                        # Stop if no clear path
                        speed_left = 0
                        speed_right = 0
                else:
                    # Continue with the Pixhawk commands if no obstacle in front
                    pass

            # Prepare DDSM commands based on the servo commands
            command_right = {
                "T": 10010,
                "id": 1,  # Right-side wheels
                "cmd": -speed_right,  # Send speed as negative for right motor
                "act": 3
            }
            command_left = {
                "T": 10010,
                "id": 2,  # Left-side wheels
                "cmd": speed_left,  # Send speed as positive for left motor
                "act": 3
            }

            # Send commands to DDSM
            ddsm_ser.write((json.dumps(command_right) + '\n').encode())
            time.sleep(0.01)  # Small delay between commands
            ddsm_ser.write((json.dumps(command_left) + '\n').encode())
            print(f"[Sent to DDSM] Right: {command_right}")
            print(f"[Sent to DDSM] Left: {command_left}")
            time.sleep(0.1)  # Maintain 10Hz loop

    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == "__main__":
    main()
