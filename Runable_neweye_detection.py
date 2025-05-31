import cv2
import mediapipe as mp
import numpy as np
import time
import serial
import serial.tools.list_ports

mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

# Eye landmarks for left and right eyes from MediaPipe face mesh
LEFT_EYE = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

def eye_aspect_ratio(landmarks, eye_indices):
    # Ensure all indices are valid
    if not all(0 <= idx < len(landmarks) for idx in eye_indices):
        return 0.0  # Return neutral value if indices are invalid
    
    # Convert landmark coordinates to NumPy arrays
    points = [np.array(landmarks[idx]) for idx in eye_indices]
    
    # Calculate Euclidean distances for vertical and horizontal landmarks
    A = np.linalg.norm(points[1] - points[5])
    B = np.linalg.norm(points[2] - points[4])
    C = np.linalg.norm(points[0] - points[3])
    
    # Compute the eye aspect ratio
    ear = (A + B) / (2.0 * C) if C != 0 else 0.0  # Avoid division by zero
    return ear

def find_working_camera(max_index=3):
    # Try different camera indices to find a working one
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            return cap, i
        cap.release()
    return None, -1

def main():
    # Initialize camera
    cap, cam_index = find_working_camera()
    if cap is None:
        print("Error: Could not open any video capture device.")
        return

    print(f"Using camera index: {cam_index}")

    # Initialize serial communication with Arduino
    arduino_port = "COM5"  # Hardcoded port for Arduino
    try:
        ser = serial.Serial(arduino_port, 9600, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset after serial connection
        print(f"Connected to Arduino on {arduino_port}")
    except serial.SerialException as e:
        print(f"Error: Could not connect to Arduino on {arduino_port}: {e}")
        print("Try: 1. Checking USB connection, 2. Verifying COM5 in Arduino IDE, 3. Closing Serial Monitor")
        cap.release()
        return

    # Initialize variables for dynamic EAR threshold and blink detection
    open_ear_values = []
    calibration_frames = 30  # Number of frames to calibrate EAR threshold
    frame_count = 0
    ear_threshold = 0.25  # Initial default threshold
    blink_count = 0
    prev_status = "Open"
    last_command = ""  # Track last command to avoid redundant sends
    log_file = open("ear_log.txt", "a")  # Open log file for appending

    with mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as face_mesh:

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break

            # Resize frame for performance optimization
            frame = cv2.resize(frame, (640, 480))
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = face_mesh.process(frame_rgb)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    h, w, _ = frame.shape
                    landmarks = []
                    for lm in face_landmarks.landmark:
                        x, y = int(lm.x * w), int(lm.y * h)
                        landmarks.append((x, y))

                    left_ear = eye_aspect_ratio(landmarks, LEFT_EYE)
                    right_ear = eye_aspect_ratio(landmarks, RIGHT_EYE)
                    ear = (left_ear + right_ear) / 2.0

                    # Dynamic EAR threshold calibration
                    if frame_count < calibration_frames and ear > 0:
                        open_ear_values.append(ear)
                        if len(open_ear_values) >= calibration_frames:
                            ear_threshold = sum(open_ear_values) / len(open_ear_values) * 0.7
                            print(f"Calibrated EAR threshold: {ear:.2f}")

                    # Determine eye status
                    eye_status = "Open" if ear > ear_threshold else "Closed"

                    # Blink detection
                    if prev_status == "Open" and eye_status == "Closed":
                        blink_count += 1
                    prev_status = eye_status

                    # Send command to Arduino ('N' for Normal, 'A' for Alert)
                    command = 'N' if eye_status == "Open" else 'A'
                    if command != last_command:
                        try:
                            ser.write(command.encode())
                            last_command = command
                            print(f"Sent to Arduino: {command}")
                        except serial.SerialException as e:
                            print(f"Error sending to Arduino: {e}")

                    # Log EAR, status, and command
                    log_file.write(f"{time.time():.2f},{ear:.2f},{eye_status},{blink_count},{command}\n")
                    log_file.flush()  # Ensure immediate write

                    # Draw eye status, blink count, and command on frame
                    display_command = "Normal" if command == 'N' else "Alert"
                    cv2.putText(frame, f'Eye: {eye_status} (EAR: {ear:.2f})', (30, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, 
                                (0, 255, 0) if eye_status == "Open" else (0, 0, 255), 2)
                    cv2.putText(frame, f'Blinks: {blink_count}', (30, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                    cv2.putText(frame, f'Command: {display_command}', (30, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

                    # Draw face mesh landmarks
                    mp_drawing.draw_landmarks(
                        image=frame, 
                        landmark_list=face_landmarks, 
                        connections=mp_face_mesh.FACEMESH_CONTOURS,
                        landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1),
                        connection_drawing_spec=mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=1))
            else:
                # Display message if no face is detected
                cv2.putText(frame, "No face detected", (30, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow('Eye State Detection', frame)

            frame_count += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Cleanup
    log_file.close()
    ser.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()