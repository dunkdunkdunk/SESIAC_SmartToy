import cv2
import mediapipe as mp
import math
import threading
import pygame
import time
import Jetson.GPIO as GPIO


# can do button press in here 
# or seperate into another service file
BUTTON_PIN = 17 # 11BCM17 **Change pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #check

print("Press button to start")
while GPIO.input(BUTTON_PIN) == GPIO.LOW:
    time.sleep(0.1)
print("Button pressed")

# IR INPUT 21BCM9, STARTER OUTPUT 23BCM11, FLIPPER RIGHT OUTPUT 31BCM6, FLIPPER LEFT OUTPUT 29BCM5
IR_PIN = 9
STARTER_PIN = 11
FLIPPER_RIGHT_PIN = 6
FLIPPER_LEFT_PIN = 5
GPIO.setup(STARTER_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_RIGHT_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_LEFT_PIN, GPIO.OUT)

cap = cv2.VideoCapture(0)

mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

ARM_CONNECTIONS = [
    (mpPose.PoseLandmark.LEFT_SHOULDER, mpPose.PoseLandmark.LEFT_ELBOW),
    (mpPose.PoseLandmark.LEFT_ELBOW, mpPose.PoseLandmark.LEFT_WRIST),
    (mpPose.PoseLandmark.RIGHT_SHOULDER, mpPose.PoseLandmark.RIGHT_ELBOW),
    (mpPose.PoseLandmark.RIGHT_ELBOW, mpPose.PoseLandmark.RIGHT_WRIST),
]

pygame.mixer.init()
EXIT_FRAME_SOUND = "./comeback.mp3"
MOVE_RIGHT_SOUND = "./right.mp3"
MOVE_LEFT_SOUND = "./left.mp3"

last_played = {
    "exit": 0,
    "left": 0,
    "right": 0,
    "solenoid_left": 0,
    "solenoid_right": 0,
    "release": 0
}
COOLDOWN = 3  # 3 seconds cooldown for sounds
GESTURE_COOLDOWN = 1.5  # 1.5 seconds cooldown for solenoid activation

def play_sound(sound_file, key):
    """Play pygame sound with cooldown"""
    global last_played
    current_time = time.time()
    if current_time - last_played[key] >= COOLDOWN:
        last_played[key] = current_time
        sound_file.play()
        print("play sound", key)

def calculate_angle(a, b, c):
    """Calculate angle between three points (shoulder, elbow, wrist)"""
    ax, ay = a
    bx, by = b
    cx, cy = c
    
    angle = math.degrees(math.atan2(cy - by, cx - bx) - math.atan2(ay - by, ax - bx))
    return abs(angle)

def send_command(action):
    # STARTER OUTPUT 23BCM11, FLIPPER RIGHT OUTPUT 31BCM6, FLIPPER LEFT OUTPUT 29BCM5
    # TODO: Gesture for start
    
    
    print(f"COMMAND: {action}")

while cap.isOpened():
    success, img = cap.read()
    if not success:
        print("Camera not detected")
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    frame_width = img.shape[1]
    current_time = time.time()

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # Get arm landmark coordinates
        left_shoulder = (landmarks[mpPose.PoseLandmark.LEFT_SHOULDER.value].x, 
                         landmarks[mpPose.PoseLandmark.LEFT_SHOULDER.value].y)
        right_shoulder = (landmarks[mpPose.PoseLandmark.RIGHT_SHOULDER.value].x, 
                          landmarks[mpPose.PoseLandmark.RIGHT_SHOULDER.value].y)

        left_wrist = (landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].x, 
                      landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].y)
        right_wrist = (landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].x, 
                       landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].y)

        # Calculate angles
        left_arm_angle = calculate_angle(left_shoulder, 
                                         (landmarks[mpPose.PoseLandmark.LEFT_ELBOW.value].x, 
                                          landmarks[mpPose.PoseLandmark.LEFT_ELBOW.value].y),
                                         left_wrist)

        right_arm_angle = calculate_angle(right_shoulder, 
                                          (landmarks[mpPose.PoseLandmark.RIGHT_ELBOW.value].x, 
                                           landmarks[mpPose.PoseLandmark.RIGHT_ELBOW.value].y),
                                          right_wrist)

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

        # **Gesture Detection for Solenoid Control**
        # Left Arm Gesture → Left Flipper
        if 80 < left_arm_angle < 120:
            if current_time - last_played["solenoid_left"] >= GESTURE_COOLDOWN:
                send_command("FLIPPER_LEFT")
                last_played["solenoid_left"] = current_time

        # Right Arm Gesture → Right Flipper
        if 80 < right_arm_angle < 120:
            if current_time - last_played["solenoid_right"] >= GESTURE_COOLDOWN:
                send_command("FLIPPER_RIGHT")
                last_played["solenoid_right"] = current_time

        # Both Arms Raised → Release Ball
        if left_wrist[1] < left_shoulder[1] and right_wrist[1] < right_shoulder[1]:
            if current_time - last_played["release"] >= GESTURE_COOLDOWN:
                send_command("RELEASE_BALL")
                last_played["release"] = current_time

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
