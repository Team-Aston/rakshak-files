import serial
import argparse
import threading
import json
import time
from pymavlink import mavutil
from dronekit import connect
from rplidar import RPLidar

# Global variables to store the latest servo output values
latest_servo1_value = 0  # Initialize to 0 (default value)
latest_servo3_value = 0  # Initialize to 0 (default value)

# Directional mapping: (angle_range_low, angle_range_high), sensor_id, orientation
DIRECTIONS = {
    'front': ((330, 360), 0, 25),  # Forward
    'right': ((60, 120), 1, 24),   # Right
    'back': ((150, 210), 2, 12),   # Back
    'left': ((240, 300), 3, 8),    # Left
}

def read_ddsm_serial(ddsm_ser):
    """ Continuously read from the DDSM serial port."""
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass  # Uncomment to debug: print(f"[DDSM] {data}", end='')
        except Exception as e:
            print(f"[Error reading DDSM] {e}")


def scale_servo_to_speed(servo_value):
    """
    Maps servo PWM range 1000-2000 µs to -200 to 200 speed units.
    """
    if servo_value is None:
        return 0
    return int((servo_value - 1735) / 500 * 200)


def lidar_loop(port, vehicle):
    """
    Reads RPLIDAR scans, segments into directions, and sends MAVLink DISTANCE_SENSOR messages.
    """
    try:
        lidar = RPLidar(port)
        print("[LIDAR] Started scanning...")

        for scan in lidar.iter_scans():
            for direction, (low, high) in DIRECTIONS.items():
                distances = []
                # Process each scan point
                for _, angle, distance in scan:  # Correct unpacking here
                    # Handle wrapping for front-facing range
                    if low <= angle <= high:
                        if distance > 0:
                            distances.append(distance)

                # Determine minimum distance or fallback
                if distances:
                    min_dist = min(distances)
                else:
                    min_dist = 1000  # If no valid distance is found, default to 1000 cm (far away)

                # Clamp to Pixhawk-compatible range (in cm)
                cm = max(20, min(1000, int(min_dist)))

                # Send MAVLink DISTANCE_SENSOR message
                vehicle.mav.distance_sensor_send(
                    int(time.time() * 1000),  # time_boot_ms (current time in milliseconds)
                    20,    # min_distance (cm)
                    1000,  # max_distance (cm)
                    cm,    # current_distance (cm)
                    0,     # sensor type: 0 = LASER
                    1,     # sensor ID (you can change this as needed)
                    0,     # orientation: 0 = forward-facing (you can adjust for other directions)
                    0      # covariance
                )
                print(f"[LIDAR -> Pixhawk] {direction.capitalize()}: {cm} cm")

            time.sleep(0.1)  # Add a small delay to prevent buffer overflow
    except Exception as e:
        print(f"[LIDAR Error] {e}")
    finally:
        lidar.stop()
        lidar.disconnect()
        print("[INFO] LIDAR stopped and disconnected.")



def main():
    parser = argparse.ArgumentParser(
        description='Bridge Pixhawk servo outputs + RPLIDAR obstacle data to DDSM driver'
    )
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0',
                        help='MAVLink port for Pixhawk (e.g., /dev/ttyAMA0)')
    parser.add_argument('--lidar_port', type=str, default='/dev/ttyUSB0',
                        help='Serial port for RPLIDAR (e.g., /dev/ttyUSB0)')
    args = parser.parse_args()

    # Connect to DDSM serial
    ddsm_ser = serial.Serial(args.ddsm_port, baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)
    threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # Connect to Pixhawk
    print("[Info] Connecting to Pixhawk...")
    vehicle = connect(args.pixhawk, wait_ready=True, baud=57600)
    print("[Info] Connected to Pixhawk.")

    # Start LIDAR scanning thread
    threading.Thread(target=lidar_loop, args=(args.lidar_port, vehicle), daemon=True).start()

    # Servo output listener
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        print(f"[Servo Output] Ch1: {latest_servo1_value} µs, Ch3: {latest_servo3_value} µs")

    try:
        while True:
            # Compute speeds
            speed_r = max(-200, min(200, scale_servo_to_speed(latest_servo1_value)))
            speed_l = max(-200, min(200, scale_servo_to_speed(latest_servo3_value)))
            print(f"[Calc] Right: {speed_r}, Left: {speed_l}")

            # Prepare commands
            cmd_r = {"T": int(time.time() * 1000), "id": 1, "cmd": -speed_r, "act": 3}
            cmd_l = {"T": int(time.time() * 1000), "id": 2, "cmd": speed_l,  "act": 3}
            ddsm_ser.write((json.dumps(cmd_r) + '\n').encode())
            time.sleep(0.01)
            ddsm_ser.write((json.dumps(cmd_l) + '\n').encode())
            print(f"[DDSM] Sent R:{cmd_r['cmd']} L:{cmd_l['cmd']}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == '__main__':
    main()
