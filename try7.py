from rplidar import RPLidar
import time

def lidar_scan(port):
    """
    Scans data from RPLIDAR and prints the angle and distance for each scan point.
    Includes delay to manage buffer size and overrun.
    """
    lidar = RPLidar(port)
    try:
        # Start scanning
        print("[INFO] Started scanning...")
        for scan in lidar.iter_scans():
            print("[SCAN] New scan:")
            for scan_point in scan:
                # The scan point contains more than 2 values (index, angle, distance, etc.)
                if len(scan_point) >= 3:
                    _, angle, distance = scan_point[:3]  # Unpack the first three values
                    print(f"Angle: {angle}°, Distance: {distance}mm")
            time.sleep(0.1)  # Sleep for 100ms to allow for buffer clearance
    except Exception as e:
        print(f"[ERROR] An error occurred: {e}")
    finally:
        lidar.stop()
        lidar.disconnect()
        print("[INFO] LIDAR stopped and disconnected.")

if __name__ == '__main__':
    lidar_port = '/dev/ttyUSB0'  # Change this to the correct port for your LIDAR
    lidar_scan(lidar_port)
