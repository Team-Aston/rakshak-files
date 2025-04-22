import serial
import argparse
import threading
from dronekit import connect
import json
import time
from rplidar import RPLidar
from math import cos, sin, radians
from pymavlink import mavutil

# Global variables to store the latest servo output values
latest_servo1_value = None
latest_servo3_value = None

# Global variable for LIDAR data
latest_distance = float('inf')  # Store the minimum distance in meters
latest_angle = 0  # Store the angle of the minimum distance

def read_ddsm_serial(ddsm_ser):
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass  # Optionally uncomment to print: print(f"[DDSM] {data}", end='')
        except Exception as e:
            print(f"[Error reading DDSM] {e}")

def read_rplidar(lidar_port):
    global latest_distance, latest_angle
    try:
        lidar = RPLidar(lidar_port)
        print("[Info] RPLIDAR initialized.")
        for scan in lidar.iter_scans(max_buf_meas=500):
            min_distance = float('inf')
            min_angle = 0
            for quality, angle, distance in scan:
                # Filter for forward-facing arc (±30 degrees around 0°)
                if -30 <= angle <= 30:
                    if distance > 0:  # Ignore invalid measurements
                        distance_m = distance / 1000.0  # Convert mm to meters
                        if distance_m < min_distance:
                            min_distance = distance_m
                            min_angle = angle
            if min_distance != float('inf'):
                latest_distance = min_distance
                latest_angle = min_angle
                print(f"[RPLIDAR] Min Distance: {latest_distance:.2f} m at {latest_angle:.1f}°")
            time.sleep(0.1)  # Control update rate
        lidar.stop()
        lidar.disconnect()
    except Exception as e:
        print(f"[Error reading RPLIDAR] {e}")

def scale_servo_to_speed(servo_value):
    """
    Maps servo PWM range 1000-2000 to -255 to 255.
    """
    if servo_value is None:
        return 0
    # Assuming servo_value is in microseconds (1000-2000)
    return int((servo_value - 1735) / 500 * 200)  # Linear mapping

def send_distance_sensor(vehicle, distance_m, angle_deg):
    """
    Send DISTANCE_SENSOR MAVLink message to Pixhawk.
    """
    # Prepare DISTANCE_SENSOR message
    msg = vehicle.message_factory.distance_sensor_encode(
        time_boot_ms=0,  # System time in ms (0 to use current time)
        min_distance=20,  # Minimum distance the sensor can measure (cm)
        max_distance=4000,  # Maximum distance the sensor can measure (cm)
        current_distance=int(distance_m * 100),  # Current distance in cm
        type=0,  # Type of sensor (0 for generic)
        id=1,  # Sensor ID
        orientation=0,  # 0 for forward-facing
        covariance=255,  # Unknown covariance
    )
    vehicle.send_mavlink(msg)

def main():
    parser = argparse.ArgumentParser(description='DroneKit to DDSM HAT and RPLIDAR Bridge with Servo Output')
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='MAVLink port (e.g., /dev/ttyAMA0)')
    parser.add_argument('--lidar-port', type=str, default='/dev/ttyUSB1', help='Serial port for RPLIDAR (e.g., /dev/ttyUSB1)')
    args = parser.parse_args()

    # Connect to DDSM serial
    ddsm_ser = serial.Serial(args.ddsm_port, baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)

    # Start thread to read from DDSM
    threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # Start thread to read from RPLIDAR
    threading.Thread(target=read_rplidar, args=(args.lidar_port,), daemon=True).start()

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

    try:
        while True:
            # Calculate speeds for right and left wheels
            speed_right = scale_servo_to_speed(latest_servo1_value)
            speed_right = max(-200, min(200, speed_right))  # Clamp to [-255, 255]
            speed_left = scale_servo_to_speed(latest_servo3_value)
            speed_left = max(-200, min(200, speed_left))  # Clamp to [-255, 255]
            print(f"[Calculated] Right Speed: {speed_right}, Left Speed: {speed_left}")

            # Create DDSM commands
            command_right = {
                "T": 10010,
                "id": 1,  # Motor ID for right-side wheels
                "cmd": -speed_right,
                "act": 3
            }
            command_left = {
                "T": 10010,
                "id": 2,  # Motor ID for left-side wheels
                "cmd": speed_left,
                "act": 3
            }

            # Prepare serialized commands
            serialized_right = (json.dumps(command_right) + '\n').encode()
            serialized_left = (json.dumps(command_left) + '\n').encode()

            # Send commands to DDSM immediately one after the other
            ddsm_ser.write(serialized_right)
            time.sleep(0.01)
            ddsm_ser.write(serialized_left)
            print(f"[Sent to DDSM] Right: {command_right}")
            print(f"[Sent to DDSM] Left: {command_left}")

            # Send LIDAR distance data to Pixhawk
            if latest_distance != float('inf'):
                send_distance_sensor(vehicle, latest_distance, latest_angle)

            time.sleep(0.1)  # Update at 10Hz
    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == "__main__":
    main()
