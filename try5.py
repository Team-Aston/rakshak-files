import serial
import argparse
import threading
from dronekit import connect
from pymavlink import mavutil
import json
import time
import math
from rplidar import RPLidar  # Assumes you're using `rplidar` Python lib

# Global variables for motor speeds
latest_servo1_value = None
latest_servo3_value = None

# Serial settings
BAUDRATE = 57600

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    return int((servo_value - 1735) / 500 * 200)

def send_distance_sensor(mav, distance_cm):
    """Send MAVLink DISTANCE_SENSOR message"""
    mav.mav.distance_sensor_send(
        int(time.time() * 1000),  # time_boot_ms
        0,                        # min_distance (cm)
        1200,                     # max_distance (cm)
        int(distance_cm),         # current_distance (cm)
        0,                        # type (0 = LASER)
        0,                        # id (arbitrary)
        25,                       # orientation (e.g., 25 = forward)
        0                         # covariance (0 = unknown)
    )

def read_lidar_loop(mav, port='/dev/ttyUSB1'):
    """Reads from RPLIDAR and sends DISTANCE_SENSOR"""
    lidar = RPLidar(port)
    print("[LIDAR] Started...")
    try:
        for scan in lidar.iter_scans():
            distances = [m[2] for m in scan if 0 < m[2] < 1200]
            if distances:
                avg_distance = sum(distances) / len(distances)
                send_distance_sensor(mav, avg_distance)
                print(f"[LIDAR -> Pixhawk] Distance: {int(avg_distance)} cm")
            time.sleep(0.1)
    except Exception as e:
        print(f"[LIDAR ERROR] {e}")
    finally:
        lidar.stop()
        lidar.disconnect()

def read_ddsm_serial(ddsm_ser):
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
        except Exception as e:
            print(f"[DDSM read error] {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('ddsm_port', type=str, help='Serial port for motor driver (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='MAVLink port (e.g., /dev/ttyAMA0)')
    parser.add_argument('--lidar_port', type=str, default='/dev/ttyUSB1', help='LIDAR serial port')
    args = parser.parse_args()

    # Motor driver serial setup
    ddsm_ser = serial.Serial(args.ddsm_port, baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)
    threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # DroneKit connection
    print("[Pixhawk] Connecting via DroneKit...")
    vehicle = connect(args.pixhawk, wait_ready=True, baud=BAUDRATE)
    print("[Pixhawk] Connected.")

    # Pymavlink connection (use same port)
    print("[MAVLink] Starting pymavlink on same port...")
    mav = mavutil.mavlink_connection(args.pixhawk, baud=BAUDRATE)
    mav.wait_heartbeat()
    print("[MAVLink] Heartbeat received.")

    # Start lidar thread
    threading.Thread(target=read_lidar_loop, args=(mav, args.lidar_port), daemon=True).start()

    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw

    try:
        while True:
            speed_right = scale_servo_to_speed(latest_servo1_value)
            speed_right = max(-200, min(200, speed_right))
            speed_left = scale_servo_to_speed(latest_servo3_value)
            speed_left = max(-200, min(200, speed_left))

            command_right = {"T": 10010, "id": 1, "cmd": -speed_right, "act": 3}
            command_left = {"T": 10010, "id": 2, "cmd": speed_left, "act": 3}

            ddsm_ser.write((json.dumps(command_right) + '\n').encode())
            time.sleep(0.01)
            ddsm_ser.write((json.dumps(command_left) + '\n').encode())
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == "__main__":
    main()