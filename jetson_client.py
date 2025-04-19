import cv2
import socketio
import time
import json

# Initialize Socket.IO client
sio = socketio.Client()

# Connect to the laptop server
try:
    sio.connect('http://172.168.2.147:3000')
    print("Connected to server")
except Exception as e:
    print(f"Connection failed: {e}")
    exit(1)

# Initialize video capture (adjust device ID if needed)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    sio.disconnect()
    exit(1)

# Set frame size for consistency
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Load Haar cascade for face detection
face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
)
if face_cascade.empty():
    print("Error: Could not load Haar cascade")
    cap.release()
    sio.disconnect()
    exit(1)

def generate_log(face_detected):
    """Generate log based on face detection."""
    message = "Intruder detected!" if face_detected else "No intruder"
    log = {"message": message, "timestamp": time.time()}
    print(f"Sending log: {log}")  # Debug log send
    return log

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            continue

        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )

        # Draw rectangles around detected faces
        face_detected = len(faces) > 0
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red rectangle

        # Encode frame to JPEG
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        # Send frame as binary
        try:
            sio.emit('video_frame', buffer.tobytes())
        except Exception as e:
            print(f"Error sending video_frame: {e}")

        # Send log message (on detection or every ~5s)
        if face_detected or time.time() % 5 < 0.1:
            try:
                log_data = generate_log(face_detected)
                sio.emit('log', log_data)
            except Exception as e:
                print(f"Error sending log: {e}")
        
        # Control frame rate (~10 FPS)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping client")
except Exception as e:
    print(f"Error: {e}")
finally:
    cap.release()
    sio.disconnect()
