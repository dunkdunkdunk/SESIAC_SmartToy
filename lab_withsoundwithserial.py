import cv2
import mediapipe as mp
import math
import threading
import playsound
import time
import serial

# Initialize camera
cap = cv2.VideoCapture(0)

# Initialize Mediapipe Pose
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Define arm connections only
ARM_CONNECTIONS = [
    (mpPose.PoseLandmark.LEFT_SHOULDER, mpPose.PoseLandmark.LEFT_ELBOW),
    (mpPose.PoseLandmark.LEFT_ELBOW, mpPose.PoseLandmark.LEFT_WRIST),
    (mpPose.PoseLandmark.RIGHT_SHOULDER, mpPose.PoseLandmark.RIGHT_ELBOW),
    (mpPose.PoseLandmark.RIGHT_ELBOW, mpPose.PoseLandmark.RIGHT_WRIST),
]

# Load sound files (Replace these with actual file paths)
EXIT_FRAME_SOUND = "./comeback.mp3"
MOVE_RIGHT_SOUND = "./right.mp3"
MOVE_LEFT_SOUND = "./left.mp3"

# Track last played sound time
last_played = {
    "exit": 0,
    "left": 0,
    "right": 0
}

# Initialize Serial communication (Change "COM3" to your Arduino port)
try:
    arduino = serial.Serial(port="COM4", baudrate=9600, timeout=1)
except Exception as e:
    print(f"Could not connect to Arduino: {e}")
    arduino = None

def play_sound(sound_file, sound_key):
    """Play sound with a 3-second delay check"""
    global last_played
    current_time = time.time()
    
    if current_time - last_played[sound_key] >= 3:  # 3-second delay
        last_played[sound_key] = current_time
        
        def sound_thread():
            try:
                playsound.playsound(sound_file)
            except Exception as e:
                print(f"Error playing sound: {e}")

        threading.Thread(target=sound_thread, daemon=True).start()

def send_command(command):
    """Send command to Arduino if connected"""
    if arduino:
        arduino.write(command.encode())  # Send command as bytes
        print(f"Sent to Arduino: {command}")

def calculate_angle(a, b, c):
    """Calculate angle between three points (shoulder, elbow, wrist)"""
    ax, ay = a
    bx, by = b
    cx, cy = c
    
    angle = math.degrees(math.atan2(cy - by, cx - bx) - math.atan2(ay - by, ax - bx))
    return abs(angle)

while cap.isOpened():
    success, img = cap.read()
    if not success:
        print("Camera not detected")
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    frame_width = img.shape[1]

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # Get arm landmark coordinates
        left_shoulder = (landmarks[mpPose.PoseLandmark.LEFT_SHOULDER.value].x, 
                         landmarks[mpPose.PoseLandmark.LEFT_SHOULDER.value].y)
        right_shoulder = (landmarks[mpPose.PoseLandmark.RIGHT_SHOULDER.value].x, 
                          landmarks[mpPose.PoseLandmark.RIGHT_SHOULDER.value].y)

        # Calculate angles
        left_arm_angle = calculate_angle(left_shoulder, 
                                         (landmarks[mpPose.PoseLandmark.LEFT_ELBOW.value].x, 
                                          landmarks[mpPose.PoseLandmark.LEFT_ELBOW.value].y),
                                         (landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].x, 
                                          landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].y))

        right_arm_angle = calculate_angle(right_shoulder, 
                                          (landmarks[mpPose.PoseLandmark.RIGHT_ELBOW.value].x, 
                                           landmarks[mpPose.PoseLandmark.RIGHT_ELBOW.value].y),
                                          (landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].x, 
                                           landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].y))

        # Get head position for frame exit detection
        nose_x = landmarks[mpPose.PoseLandmark.NOSE.value].x

        # Sound alert if person moves out of frame
        if nose_x < 0.1 or nose_x > 0.9:  # Almost out of frame
            play_sound(EXIT_FRAME_SOUND, "exit")

        # Sound alert for left/right movement
        if nose_x < 0.3:  # Too much left
            play_sound(MOVE_LEFT_SOUND, "left")
        elif nose_x > 0.7:  # Too much right
            play_sound(MOVE_RIGHT_SOUND, "right")

        # Control servo motors based on arm angles
        if 60 < left_arm_angle < 120:
            print("Left")
            send_command("L")  # Send 'L' to Arduino
        
        if 60 < right_arm_angle < 120:
            print("Right")
            send_command("R")  # Send 'R' to Arduino

        # Draw only arm landmarks and connections
        for connection in ARM_CONNECTIONS:
            start = landmarks[connection[0].value]
            end = landmarks[connection[1].value]

            start_point = (int(start.x * img.shape[1]), int(start.y * img.shape[0]))
            end_point = (int(end.x * img.shape[1]), int(end.y * img.shape[0]))

            cv2.line(img, start_point, end_point, (0, 255, 0), 3)  # Green lines
            cv2.circle(img, start_point, 5, (0, 0, 255), -1)  # Red dots
            cv2.circle(img, end_point, 5, (0, 0, 255), -1)

    else:
        # No person detected → Play exit sound
        play_sound(EXIT_FRAME_SOUND, "exit")

    cv2.imshow("Arm Tracking with Sound Alerts", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if arduino:
    arduino.close()
