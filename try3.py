import serial
import argparse
import threading
from dronekit import connect
import json
import time

def read_ddsm_serial(ddsm_ser):
    while True:
        try:
            data = ddsm_ser.readline().decode('utf-8', errors='ignore')
            if data:
                pass 
                # print(f"[DDSM] {data}", end='')
        except Exception as e:
            print(f"[Error reading DDSM] {e}")

def scale_rc_to_speed(rc_value):
    """
    Maps RC input range 1000-2000 to -255 to 255.
    """
    if rc_value is None:
        return 0
    return int((rc_value - 1500) / 500 * 200)

def main():
    parser = argparse.ArgumentParser(description='DroneKit to DDSM HAT Bridge')
    parser.add_argument('ddsm_port', type=str, help='Serial port for DDSM HAT (e.g., /dev/ttyUSB0)')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='MAVLink port (e.g., /dev/ttyAMA0)')
    args = parser.parse_args()

    # Connect to DDSM serial
    ddsm_ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    ddsm_ser.setRTS(False)
    ddsm_ser.setDTR(False)

    # Start thread to read from DDSMa
    threading.Thread(target=read_ddsm_serial, args=(ddsm_ser,), daemon=True).start()

    # Connect to Pixhawk
    print("[Info] Connecting to Pixhawk...")
    vehicle = connect("/dev/ttyACM1", wait_ready=True, baud=57600)
    print("[Info] Connected to Pixhawk.")

    try:
        while True:
            # Example: Read RC channel 3 (Throttle) or channel 2 (Pitch) as speed control
            rc_value = vehicle.channels.get('3') # Change to desired RC channel
            print("rc:",rc_value)
            speed = scale_rc_to_speed(rc_value)
            speed = max(-255, min(255, speed)) # Clamp to [-255, 255]
            print("speed:" , speed)
            command = {
                "T": 10010,
                "id": 1,
                "cmd": speed,
                "act": 3
            }

            ddsm_ser.write((json.dumps(command) + '\n').encode())
            print(f"[Sent to DDSM] {command}")
            time.sleep(0.1) # Send at 10Hz
    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        vehicle.close()
        ddsm_ser.close()

if __name__ == "__main__":
    main()
