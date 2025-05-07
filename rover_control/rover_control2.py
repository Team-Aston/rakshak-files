import serial
import argparse
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
import json
import time
from rplidar import RPLidar
import pygame
import math

# Global variables
latest_servo1_value = None
latest_servo3_value = None
latest_scan = None
MIN_DISTANCE = 500  # mm, obstacle distance threshold
ACCEPTANCE_RADIUS = 2.0  # meters, waypoint acceptance radius
MAX_SPEED = 1.0  # m/s, navigation speed
CHECK_INTERVAL = 0.5  # seconds, update interval
obstacle_detected = False
last_switch_position = None  # For RC channel 8 audio logic
control_mode = "MANUAL"  # Start in manual mode
AVOIDANCE_OFFSET = 2.0  # meters, distance for temporary waypoint
EARTH_RADIUS = 6371000  # meters

# Calculate distance between two GPS coordinates (in meters)
def get_distance_meters(location1, location2):
    dlat = math.radians(location2.lat - location1.lat)
    dlon = math.radians(location2.lon - location1.lon)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(location1.lat)) * math.cos(math.radians(location2.lat)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_RADIUS * c

# Calculate a new waypoint offset from current position
def calculate_offset_waypoint(current_location, heading, offset_distance, direction):
    # direction: +90 for left, -90 for right
    heading_rad = math.radians(heading + direction)
    lat_offset = (offset_distance * math.cos(heading_rad)) / EARTH_RADIUS
    lon_offset = (offset_distance * math.sin(heading_rad)) / (EARTH_RADIUS * math.cos(math.radians(current_location.lat)))
    new_lat = current_location.lat + math.degrees(lat_offset)
    new_lon = current_location.lon + math.degrees(lon_offset)
    return LocationGlobalRelative(new_lat, new_lon, 0)

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

def process_scan(scan):
    front_clear = True
    left_clear = True
    right_clear = True
    for (_, angle, distance) in scan:
        if distance == 0:
            continue
        if (angle >= 350 or angle <= 10) and distance < MIN_DISTANCE:
            front_clear = False
        elif 60 <= angle <= 120 and distance < MIN_DISTANCE:
            right_clear = False
        elif 240 <= angle <= 300 and distance < MIN_DISTANCE:
            left_clear = False
    return front_clear, left_clear, right_clear

def lidar_thread(lidar):
    global latest_scan
    try:
        for scan in lidar.iter_scans():
            latest_scan = scan
    except Exception as e:
        print(f"[LiDAR Error] {e}")

def obstacle_monitor():
    global obstacle_detected, latest_scan
    while True:
        if latest_scan is not None:
            front_clear, _, _ = process_scan(latest_scan)
            obstacle_detected = not front_clear
        time.sleep(0.1)

def play_audio(file_path):
    try:
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
    except Exception as e:
        print(f"[Audio Error] {e}")

def main():
    parser = argparse.ArgumentParser:description='Rover Control with RC and Autonomous Modes')
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='Pixhawk MAVLink port')
    parser.add_argument('--lidar', type=str, default='/dev/ttyUSB0', help='LiDAR port')
    args = parser.parse_args()

    # Initialize pygame for audio
    pygame.mixer.init()

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

    # Connect to LiDAR
    lidar = RPLidar(args.lidar)
    threading.Thread(target=lidar_thread, args=(lidar,), daemon=True).start()

    # Start obstacle monitoring thread
    threading.Thread(target=obstacle_monitor, daemon=True).start()

    # Servo output listener
    @vehicle.on_message('SERVO_OUTPUT_RAW')
    def servo_listener(self, name, message):
        global latest_servo1_value, latest_servo3_value
        latest_servo1_value = message.servo1_raw
        latest_servo3_value = message.servo3_raw
        print(f"[Servo Output] Channel 1: {latest_servo1_value} µs, Channel 3: {latest_servo3_value} µs")

    # Define waypoints (replace with your coordinates)
    waypoints = [
        LocationGlobalRelative(37.7749, -122.4194, 0),
        LocationGlobalRelative(37.7750, -122.4195, 0),
    ]

    try:
        # Set initial mode to MANUAL
        vehicle.mode = VehicleMode("MANUAL")
        while vehicle.mode.name != "MANUAL":
            print("Waiting for MANUAL mode...")
            time.sleep(1)

        while True:
            # Check RC channel 6 for mode toggle
            rc_value = vehicle.channels.get('6')
            global control_mode
            if rc_value is not None and rc_value != 0:
                if rc_value < 1300:
                    new_mode = "MANUAL"
                else:
                    new_mode = "GUIDED"
                
                if new_mode != control_mode:
                    print(f"[Mode Change] Switching to {new_mode}")
                    vehicle.mode = VehicleMode(new_mode)
                    control_mode = new_mode
                    if new_mode == "GUIDED":
                        vehicle.armed = True
                        while not vehicle.armed:
                            print("Waiting to arm...")
                            time.sleep(1)
                        while vehicle.gps_0.fix_type < 3:
                            print("Waiting for GPS lock... Fix type:", vehicle.gps_0.fix_type)
                            time.sleep(1)

            # Retrieve and process servo values
            servo1 = latest_servo1_value
            servo3 = latest_servo3_value
            speed_right = scale_servo_to_speed(servo1)
            speed_left = scale_servo_to_speed(servo3)
            speed_right = max(-100, min(100, speed_right))
            speed_left = max(-100, min(100, speed_left))

            # RC Channel 8 audio logic
            rc_value_8 = vehicle.channels.get('8')
            global last_switch_position
            if rc_value_8 is not None and rc_value_8 != 0:
                if rc_value_8 < 1300:
                    switch_position = 'low'
                elif 1300 <= rc_value_8 <= 1700:
                    switch_position = 'mid'
                else:
                    switch_position = 'high'
                if switch_position != last_switch_position:
                    print(f"[RC Switch] Channel 8 in {switch_position} position")
                    if switch_position == 'low':
                        play_audio('switch_low.mp3')
                    elif switch_position == 'mid':
                        play_audio('switch_mid.mp3')
                    elif switch_position == 'high':
                        play_audio('switch_high.mp3')
                    last_switch_position = switch_position

            # Obstacle avoidance logic
            avoidance_active = False
            if speed_left > 0 and speed_right > 0 and latest_scan is not None:
                front_clear, left_clear, right_clear = process_scan(latest_scan)
                if not front_clear:
                    avoidance_active = True
                    print("[Obstacle Detected] Initiating avoidance...")
                    if left_clear:
                        print("[Action] Steering left")
                        if control_mode == "MANUAL":
                            speed_left = int(speed_left * 0.5)  # Reduce left speed to turn left
                            speed_right = speed_right
                        else:  # GUIDED mode
                            current_location = vehicle.location.global_relative_frame
                            heading = vehicle.heading
                            temp_wp = calculate_offset_waypoint(current_location, heading, AVOIDANCE_OFFSET, 90)
                            vehicle.simple_goto(temp_wp, groundspeed=MAX_SPEED)
                    elif right_clear:
                        print("[Action] Steering right")
                        if control_mode == "MANUAL":
                            speed_left = speed_left
                            speed_right = int(speed_right * 0.5)  # Reduce right speed to turn right
                        else:  # GUIDED mode
                            current_location = vehicle.location.global_relative_frame
                            heading = vehicle.heading
                            temp_wp = calculate_offset_waypoint(current_location, heading, AVOIDANCE_OFFSET, -90)
                            vehicle.simple_goto(temp_wp, groundspeed=MAX_SPEED)
                    else:
                        print("[Action] Stopping - No path clear")
                        speed_left = 0
                        speed_right = 0
                        if control_mode == "GUIDED":
                            current_location = vehicle.location.global_relative_frame
                            vehicle.simple_goto(current_location, groundspeed=0)

            # Autonomous navigation (GUIDED mode)
            if control_mode == "GUIDED" and not avoidance_active:
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

                        # Update motor commands
                        servo1 = latest_servo1_value
                        servo3 = latest_servo3_value
                        speed_right = scale_servo_to_speed(servo1)
                        speed_left = scale_servo_to_speed(servo3)
                        speed_right = max(-100, min(100, speed_right))
                        speed_left = max(-100, min(100, speed_left))

                        # Obstacle avoidance handled above
                        if avoidance_active:
                            # Wait for avoidance to complete
                            while avoidance_active and latest_scan is not None:
                                front_clear, left_clear, right_clear = process_scan(latest_scan)
                                avoidance_active = not front_clear and (speed_left > 0 and speed_right > 0)
                                time.sleep(0.5)
                            print("[Obstacle cleared] Resuming navigation...")
                            vehicle.simple_goto(wp, groundspeed=MAX_SPEED)

                        # Send motor commands
                        command_right = {"T": 10010, "id": 1, "cmd": -speed_right, "act": 3}
                        command_left = {"T": 10010}
                        command_left = {"T": 10010, "id": 2, "cmd": speed_left, "act": 3}
                        ddsm_ser.write((json.dumps(command_right) + '\n').encode())
                        time.sleep(0.01)
                        ddsm_ser.write((json.dumps(command_left) + '\n').encode())
                        print(f"[Sent to DDSM] Right: {command_right}")
                        print(f"[Sent to DDSM] Left: {command_left}")

                        time.sleep(CHECK_INTERVAL)

                    time.sleep(2)  # Stabilize before next waypoint

                # After waypoints, switch to RTL
                vehicle.mode = VehicleMode("RTL")
                control_mode = "RTL"

            # Manual control (MANUAL mode)
            else:
                # Send motor commands
                command_right = {"T": 10010, "id": 1, "cmd": -speed_right, "act": 3}
                command_left = {"T": 10010, "id": 2, "cmd": speed_left, "act": 3}
                ddsm_ser.write((json.dumps(command_right) + '\n').encode())
                time.sleep(0.01)
                ddsm_ser.write((json.dumps(command_left) + '\n').encode())
                print(f"[Sent to DDSM] Right: {command_right}")
                print(f"[Sent to DDSM] Left: {command_left}")

            time.sleep(0.1)  # Maintain ~10Hz loop for manual mode

    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        pygame.mixer.quit()

if __name__ == "__main__":
    main()