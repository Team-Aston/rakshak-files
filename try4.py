import serial
import argparse
import threading
from dronekit import connect
import json
import time

# Global variables to store the latest servo output values
latest_servo1_value = None
latest_servo3_value = None

def read_ddsm_serial(ddsm_ser):
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass  # Optionally uncomment to print: print(f"[DDSM] {data}", end='')
        except Exception as e:
            print(f"[Error reading DDSM] {e}")

def scale_servo_to_speed(servo_value):
    """
    Maps servo PWM range 1000-2000 to -255 to 255.
    """
    if servo_value is None:
        return 0
    # Assuming servo_value is in microseconds (1000-2000)
    return int((servo_value - 1735) / 500 * 200)  # Linear mapping

def main():
    parser = argparse.ArgumentParser(description='DroneKit to DDSM HAT Bridge with Servo Output')
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='MAVLink port (e.g., /dev/ttyAMA0)')
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
            time.sleep(0.1)  # Send at 10Hz
    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == "__main__":
    main()
