from dronekit import connect
import time
import json
import serial
import threading
import argparse

# Parse command-line arguments for serial port
parser = argparse.ArgumentParser(description='Pixhawk to DDSM Driver Communication')
parser.add_argument('--port', type=str, default='/dev/ttyACM2', help='Serial port for DDSM driver (e.g., /dev/ttyACM2)')
parser.add_argument('--pixhawk', type=str, default='/dev/ttyACM0', help='Serial port for Pixhawk (e.g., /dev/ttyACM0)')
args = parser.parse_args()

# Connect to Pixhawk
print("Connecting to Pixhawk...")
vehicle = connect(args.pixhawk, baud=57600, wait_ready=True)
print("Connected to Pixhawk.")

# Setup serial communication for DDSM driver
driver_serial = serial.Serial(args.port, baudrate=115200, dsrdtr=None, timeout=1)
driver_serial.setRTS(False)
driver_serial.setDTR(False)

# Serial read thread to monitor DDSM driver responses
def read_serial():
    while True:
        data = driver_serial.readline().decode('utf-8', errors='ignore').strip()
        if data:
            print(f"DDSM Response: {data}")

# Start serial read thread
serial_recv_thread = threading.Thread(target=read_serial)
serial_recv_thread.daemon = True
serial_recv_thread.start()

def convert_to_json(servo_data):
    try:
        # Extract left and right motor speeds
        left_speed = servo_data.servo1_raw
        right_speed = servo_data.servo3_raw

        # Map Pixhawk PWM (1000-2000) to DDSM driver range (adjust as needed)
        left_speed = max(min(int(left_speed), 2000), 1000)  # Clamp values
        right_speed = max(min(int(right_speed), 2000), 1000)

        json_data = {
            "left_motor": left_speed,
            "right_motor": right_speed
        }

        print(f"JSON Data: {json.dumps(json_data)}")
        return json_data
    except AttributeError as e:
        print(f"Error accessing servo data: {e}")
        return None

def send_to_ddsm(left_speed, right_speed):
    try:
        # Adjust scaling if needed (example: map 1000-2000 PWM to -100 to 100 for DDSM)
        ddsm_left = (left_speed - 1500) // 5  # Example scaling, adjust per DDSM requirements
        ddsm_right = (right_speed - 1500) // 5

        print(f"Sending to DDSM: Left speed = {ddsm_left}, Right speed = {ddsm_right}")

        # Format commands for DDSM driver
        left_command = f'{{"T":1,"ID":1,"cmd":{ddsm_left},"act":3}}'
        right_command = f'{{"T":1,"ID":2,"cmd":{ddsm_right},"act":3}}'

        print(f"Left command: {left_command}")
        print(f"Right command: {right_command}")

        # Send commands with proper timing
        driver_serial.write((left_command + '\n').encode('utf-8'))
        time.sleep(0.01)
        driver_serial.write((right_command + '\n').encode('utf-8'))
        driver_serial.flush()  # Ensure data is sent
    except Exception as e:
        print(f"Error sending to DDSM: {e}")

def handle_servo_output(vehicle, name, message):
    # Process servo output message
    json_data = convert_to_json(message)
    if json_data:
        left_speed = json_data['left_motor']
        right_speed = json_data['right_motor']
        send_to_ddsm(left_speed, right_speed)

# Add message listener for 'SERVO_OUTPUT_RAW'
vehicle.add_message_listener('SERVO_OUTPUT_RAW', handle_servo_output)

try:
    print("Running. Press Ctrl+C to exit.")
    while True:
        time.sleep(1)  # Keep script alive to process messages
except KeyboardInterrupt:
    print("\nShutting down...")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    print("Closing connections...")
    vehicle.close()
    driver_serial.close()
