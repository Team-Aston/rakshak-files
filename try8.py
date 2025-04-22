import serial
import argparse
import threading
import time
from rplidar import RPLidar, RPLidarException
from pymavlink import mavutil

# Global variables for LIDAR data
latest_distance = float('inf')  # Minimum distance in meters
latest_angle = 0  # Angle of minimum distance

def read_rplidar(lidar_port, baudrate=115200):
    """Read RPLIDAR data and update minimum distance in forward arc."""
    global latest_distance, latest_angle
    lidar = None
    scan_count = 0
    while True:
        try:
            print(f"[Info] Initializing RPLIDAR on {lidar_port} with baudrate {baudrate}...")
            lidar = RPLidar(lidar_port, baudrate=baudrate, timeout=1)
            print("[Info] RPLIDAR initialized. Health:", lidar.get_health())
            print("[Info] RPLIDAR Info:", lidar.get_info())
            for scan in lidar.iter_scans(max_buf_meas=500, min_len=5):
                scan_count += 1
                min_distance = float('inf')
                min_angle = 0
                valid_points = 0
                for scan_point in scan:
                    if len(scan_point) >= 3:
                        _, angle, distance = scan_point[:3]
                        if -30 <= angle <= 30 and distance > 0:
                            valid_points += 1
                            distance_m = distance / 1000.0  # Convert mm to meters
                            if distance_m < min_distance:
                                min_distance = distance_m
                                min_angle = angle
                if min_distance != float('inf') and valid_points > 0:
                    latest_distance = min_distance
                    latest_angle = min_angle
                    print(f"[RPLIDAR] Scan {scan_count}: Min Distance: {latest_distance:.2f} m at {latest_angle:.1f}° ({valid_points} valid points)")
                else:
                    print(f"[RPLIDAR] Scan {scan_count}: No valid points in forward arc")
                    latest_distance = float('inf')  # Reset if no valid points
                time.sleep(0.1)  # Match standalone script
                if lidar._serial.in_waiting > 1000:
                    print(f"[Warning] High buffer load: {lidar._serial.in_waiting} bytes. Clearing...")
                    lidar._serial.reset_input_buffer()
        except RPLidarException as e:
            print(f"[RPLIDAR Error] {e}. Buffer: {lidar._serial.in_waiting if lidar else 'N/A'} bytes. Retrying in 5 seconds...")
            if lidar:
                try:
                    lidar.stop()
                    lidar.disconnect()
                except:
                    pass
            time.sleep(5)
        except Exception as e:
            print(f"[Unexpected RPLIDAR Error] {e}. Retrying in 5 seconds...")
            if lidar:
                try:
                    lidar.stop()
                    lidar.disconnect()
                except:
                    pass
            time.sleep(5)

def send_distance_sensor(master, distance_m, angle_deg):
    """Send DISTANCE_SENSOR MAVLink message to Pixhawk."""
    master.mav.distance_sensor_send(
        time_boot_ms=0,
        min_distance=20,  # 0.2m in cm
        max_distance=1200,  # 12m in cm (RPLIDAR A1 range)
        current_distance=int(distance_m * 100),  # Convert m to cm
        type=0,  # Generic
        id=1,  # Sensor ID
        orientation=0,  # Forward-facing
        covariance=255  # Unknown
    )

def main():
    parser = argparse.ArgumentParser(description='Jetson RPLIDAR to Pixhawk for Obstacle Avoidance')
    parser.add_argument('--pixhawk', type=str, default='/dev/ttyAMA0', help='MAVLink port (e.g., /dev/ttyAMA0)')
    parser.add_argument('--lidar-port', type=str, default='/dev/ttyUSB0', help='Serial port for RPLIDAR (e.g., /dev/ttyUSB0)')
    parser.add_argument('--lidar-baudrate', type=int, default=115200, help='Baudrate for RPLIDAR (e.g., 115200)')
    parser.add_argument('--mavlink-baud', type=int, default=57600, help='Baudrate for MAVLink (e.g., 57600)')
    args = parser.parse_args()

    # Start RPLIDAR thread
    threading.Thread(target=read_rplidar, args=(args.lidar_port, args.lidar_baudrate), daemon=True).start()

    # Connect to Pixhawk via MAVLink
    print("[Info] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(args.pixhawk, baud=args.mavlink_baud)
    master.wait_heartbeat()
    print("[Info] Connected to Pixhawk.")

    try:
        while True:
            # Send LIDAR distance data to Pixhawk
            if latest_distance != float('inf'):
                send_distance_sensor(master, latest_distance, latest_angle)
                print(f"[Pixhawk] Sent DISTANCE_SENSOR: {latest_distance:.2f} m")
            else:
                print("[Pixhawk] No valid distance data to send")
            time.sleep(0.1)  # Update at 10Hz
    except KeyboardInterrupt:
        print("\n[Info] Shutting down...")
    finally:
        master.close()

if __name__ == "__main__":
    main()
