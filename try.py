from dronekit import connect
import time
import json
import serial

# Connect to Pixhawk
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

# Setup serial communication for DDSM driver
driver_serial = serial.Serial('/dev/ttyACM2', 115200, timeout=1)

print("Connected to Pixhawk.")

def convert_to_json(servo_data):
    # Extract left and right motor speeds
    left_speed = servo_data.servo1_raw
    right_speed = servo_data.servo3_raw
   
    # Create the JSON data to send
    json_data = {
        "left_motor": left_speed,
        "right_motor": right_speed
    }
   
    # Debug: Print the JSON data
    print(f"JSON Data to send: {json.dumps(json_data)}")
   
    return json_data

def send_to_ddsm(left_speed, right_speed):
    # Send the motor control data to the DDSM driver as JSON-like command
    driver_serial.write(f'{{"T":1,"ID":1,"cmd":{int(left_speed)},"act":3}}'.encode())  # Left motor
    driver_serial.write(f'{{"T":1,"ID":2,"cmd":{int(right_speed)},"act":3}}'.encode())  # Right motor

def handle_servo_output(self, name, message):
    # Convert received servo output into JSON
    json_data = convert_to_json(message)
    left_speed = json_data['left_motor']
    right_speed = json_data['right_motor']
   
    print(f"Sending to DDSM driver: Left speed = {left_speed}, Right speed = {right_speed}")
   
    # Send the data to the DDSM driver
    send_to_ddsm(left_speed, right_speed)

# Add message listener for 'SERVO_OUTPUT_RAW'
vehicle.add_message_listener('SERVO_OUTPUT_RAW', handle_servo_output)

try:
    while True:
        time.sleep(1)  # Keep the script running to process messages
except KeyboardInterrupt:
    print("Shutting down.")
    vehicle.close()
    driver_serial.close()  # Close the serial connection to DDSM driver
