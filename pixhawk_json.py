from dronekit import connect
import time
import json
import serial

# Connect to Pixhawk
try:
    vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
except Exception as e:
    print(f"Failed to connect to Pixhawk: {e}")
    exit(1)

# Setup serial communication for DDSM driver
try:
    driver_serial = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
except Exception as e:
    print(f"Failed to connect to DDSM driver: {e}")
    exit(1)

print("Connected to Pixhawk and DDSM driver.")

def map_pwm_to_speed(pwm):
    """
    Map PWM (1000-2000) to DDSM speed (-100 to 100).
    Adjust output range based on driver requirements.
    """
    speed = ((pwm - 1500) / 500) * 100
    return max(min(int(speed), 100), -100)

def initialize_ddsm():
    """Initialize DDSM driver by checking IDs and setting mode."""
    try:
        # Check motor IDs
        cmd_id_check = {"T": 10031}
        driver_serial.write((json.dumps(cmd_id_check) + '\n').encode())
        time.sleep(0.1)
       
        # Set mode for motors (mode 2 as example)
        cmd_mode = {"T": 10012, "id": 1, "mode": 2}
        driver_serial.write((json.dumps(cmd_mode) + '\n').encode())
        cmd_mode["id"] = 2
        driver_serial.write((json.dumps(cmd_mode) + '\n').encode())
       
        print("DDSM initialized.")
    except Exception as e:
        print(f"Error initializing DDSM: {e}")

def send_to_ddsm(left_speed, right_speed):
    """Send motor control commands to DDSM driver."""
    try:
        # Left motor (ID 1)
        cmd_left = {"T": 10010, "id": 1, "cmd": left_speed, "act": 3}
        driver_serial.write((json.dumps(cmd_left) + '\n').encode())
       
        # Right motor (ID 2)
        cmd_right = {"T": 10010, "id": 2, "cmd": right_speed, "act": 3}
        driver_serial.write((json.dumps(cmd_right) + '\n').encode())
       
        print(f"Sent to DDSM: Left speed = {left_speed}, Right speed = {right_speed}")
    except Exception as e:
        print(f"Error sending to DDSM: {e}")

def handle_servo_output(name, message):
    """Callback for servo output messages."""
    try:
        # Extract left and right motor PWM values
        left_pwm = message.servo1_raw
        right_pwm = message.servo3_raw
       
        # Validate PWM values
        if not (1000 <= left_pwm <= 2000 and 1000 <= right_pwm <= 2000):
            print(f"Invalid PWM values: Left={left_pwm}, Right={right_pwm}")
            return
       
        # Convert PWM to speed
        left_speed = map_pwm_to_speed(left_pwm)
        right_speed = map_pwm_to_speed(right_pwm)
       
        # Send to DDSM driver
        send_to_ddsm(left_speed, right_speed)
    except Exception as e:
        print(f"Error processing servo output: {e}")

# Initialize DDSM driver
initialize_ddsm()

# Add message listener for 'SERVO_OUTPUT_RAW'
vehicle.add_message_listener('SERVO_OUTPUT_RAW', handle_servo_output)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down.")
finally:
    vehicle.close()
    driver_serial.close()
