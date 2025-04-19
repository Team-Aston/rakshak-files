import cv2
import socketio
import time
import os
from deepface import DeepFace
import numpy as np

# Initialize Socket.IO client
sio = socketio.Client()

# Connect to the laptop server
try:
    sio.connect('http://172.168.2.147:3000')
    print("Connected to server")
except Exception as e:
    print(f"Connection failed: {e}")
    exit(1)

# Initialize video capture
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    sio.disconnect()
    exit(1)

# Set frame size
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Load Haar cascade for face detection
face_cascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
if face_cascade.empty():
    print("Error: Could not load Haar cascade")
    cap.release()
    sio.disconnect()
    exit(1)

# Load known faces
known_faces_dir = "known_faces"
known_faces = []
known_names = []
if os.path.exists(known_faces_dir):
    for filename in os.listdir(known_faces_dir):
        if filename.endswith((".jpg", ".png")):
            filepath = os.path.join(known_faces_dir, filename)
            known_faces.append(cv2.imread(filepath))
            known_names.append(os.path.splitext(filename)[0])
            print(f"Loaded known face: {filename}")
else:
    print("Warning: known_faces/ directory not found")
    os.makedirs(known_faces_dir)

# Log control
last_log_time = 0
last_face_id = None
log_cooldown = 30  # Seconds before re-logging same face

def identify_face(face_img):
    """Compare face against known faces using DeepFace."""
    if not known_faces:
        return "Unknown (Intruder)"
    
    try:
        for i, known_img in enumerate(known_faces):
            # Convert images to RGB for DeepFace
            face_rgb = cv2.cvtColor(face_img, cv2.COLOR_BGR2RGB)
            known_rgb = cv2.cvtColor(known_img, cv2.COLOR_BGR2RGB)
            # Verify similarity
            result = DeepFace.verify(
                face_rgb,
                known_rgb,
                model_name="VGG-Face",
                detector_backend="skip",  # Already cropped
                enforce_detection=False
            )
            if result["verified"]:
                return known_names[i]
        return "Unknown (Intruder)"
    except Exception as e:
        print(f"Face verification error: {e}")
        return "Unknown (Intruder)"

def generate_log(face_id):
    """Generate log based on face identity."""
    message = f"Detected: {face_id}"
    log = {"message": message, "timestamp": time.time()}
    print(f"Sending log: {log}")
    return log

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            continue

        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )

        # Process detected faces
        current_face_id = None
        for (x, y, w, h) in faces:
            # Draw rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # Crop face for recognition
            face_img = frame[y:y+h, x:x+w]
            # Identify face
            face_id = identify_face(face_img)
            current_face_id = face_id
            # Add label
            cv2.putText(
                frame, face_id, (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2
            )

        # Encode frame to JPEG
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        # Send frame as binary
        try:
            sio.emit('video_frame', buffer.tobytes())
        except Exception as e:
            print(f"Error sending video_frame: {e}")

        # Send log if new face or cooldown elapsed
        current_time = time.time()
        if current_face_id:
            if (current_face_id != last_face_id or 
                current_time - last_log_time >= log_cooldown):
                try:
                    log_data = generate_log(current_face_id)
                    sio.emit('log', log_data)
                    last_log_time = current_time
                    last_face_id = current_face_id
                except Exception as e:
                    print(f"Error sending log: {e}")
        elif last_face_id:  # No face detected, reset if previously logged
            log_data = generate_log("No face detected")
            try:
                sio.emit('log', log_data)
                last_log_time = current_time
                last_face_id = None
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
