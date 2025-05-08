import serial
import argparse
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
import json
import time
import math

# Global variables
latest_servo1_value = None
latest_servo3_value = None
ACCEPTANCE_RADIUS = 2.0  # meters, waypoint acceptance radius
MAX_SPEED = 1.0  # m/s, navigation speed
CHECK_INTERVAL = 0.5  # seconds, update interval
EARTH_RADIUS = 6371000  # meters

# Calculate distance between two GPS coordinates (in meters)
def get_distance_meters(location1, location2):
    dlat = math.radians(location2.lat - location1.lat)
    dlon = math.radians(location2.lon - location1.lon)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(location1.lat)) * math.cos(math.radians(location2.lat)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_RADIUS * c

def read_ddsm_serial(ddsm_ser):
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass  # Optionally print: print(f"[DDSM] {data}", end='')
        except Exception as e:
            print(f"[Error reading DDSM] {e}")

def scale_servo_to_speed(servo_value):
    if servo_value is None:
        return 0
    # Maps servo_value (1000-2000 µs) to -100 to 100, centered at 1500
    return int((servo_value - 1500) / 500 * 100)

def main():
    parser = argparse.ArgumentParser(description='Rover Waypoint Navigation')
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='Pixhawk MAVLink port')
    args = parser.parse_args()

    # Connect to DDSM serial
    ddsm_ser = serial.Serial(args.ddsm_port, baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)

    # Start DDSM read thread
    threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # Connect to Pixhawk
    print("[Info] Connecting to Pixhawk...")
    vehicle = connect(args.pixhawk, wait_ready=True, baud=57600)
    print("[Info] Connected to Pixhawk.")

    # Servo output listener
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        print(f"[Servo Output] Channel 1: {latest_servo1_value} µs, Channel 3: {latest_servo3_value} µs")

    # Define waypoints (replace with your coordinates)
    waypoints = [
        LocationGlobalRelative(17.3970892, 78.4901811, 0),
        LocationGlobalRelative(17.3969139, 78.4902643, 0),
    ]

    try:
        # Set GUIDED mode and arm
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            print("Waiting for GUIDED mode...")
            time.sleep(1)
        vehicle.armed = True
        while not vehicle.armed:
            print("Waiting to arm...")
            time.sleep(1)
        while vehicle.gps_0.fix_type < 3:
            print("Waiting for GPS lock... Fix type:", vehicle.gps_0.fix_type)
            time.sleep(1)

        # Navigate waypoints
        for wp in waypoints:
            print(f"Navigating to waypoint: {wp.lat}, {wp.lon}")
            vehicle.simple_goto(wp, groundspeed=MAX_SPEED)

            while True:
                current_location = vehicle.location.global_relative_frame
                distance = get_distance_meters(current_location, wp)
                print(f"Distance to waypoint: {distance:.2f} meters")

                if distance <= ACCEPTANCE_RADIUS:
                    print("Waypoint reached!")
                    break

                # Read servo values and send motor commands
                servo1 = latest_servo1_value
                servo3 = latest_servo3_value
                speed_right = scale_servo_to_speed(servo1)
                speed_left = scale_servo_to_speed(servo3)
                speed_right = max(-100, min(100, speed_right))
                speed_left = max(-100, min(100, speed_left))

                # Send JSON commands to DDSM
                command_right = {"T": 10010, "id": 1, "cmd": -speed_right, "act": 3}
                command_left = {"T": 10010, "id": 2, "cmd": speed_left, "act": 3}
                ddsm_ser.write((json.dumps(command_right) + '\n').encode())
                time.sleep(0.01)
                ddsm_ser.write((json.dumps(command_left) + '\n').encode())
                print(f"[Sent to DDSM] Right: {command_right}")
                print(f"[Sent to DDSM] Left: {command_left}")

                time.sleep(CHECK_INTERVAL)

            time.sleep(2)  # Stabilize before next waypoint

        # Switch to RTL after waypoints
        print("[Info] Waypoints complete. Switching to RTL...")
        vehicle.mode = VehicleMode("RTL")
        while vehicle.mode.name != "RTL":
            print("Waiting for RTL mode...")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == "__main__":
    main()
