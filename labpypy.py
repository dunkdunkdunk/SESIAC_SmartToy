import cv2
import mediapipe as mp
import math
import time
import pygame
import Jetson.GPIO as GPIO

# IR INPUT 21BCM9, STARTER OUTPUT 23BCM11, FLIPPER RIGHT OUTPUT 31BCM6, FLIPPER LEFT OUTPUT 29BCM5
BUTTON_PIN = 17
IR_PIN = 9
STARTER_PIN = 11
FLIPPER_RIGHT_PIN = 6
FLIPPER_LEFT_PIN = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(STARTER_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_RIGHT_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_LEFT_PIN, GPIO.OUT)

# # Wait for button press to start
# print("Press button to start")
# while GPIO.input(BUTTON_PIN) == GPIO.LOW:
#     time.sleep(0.1)
# print("Button pressed!")

# Setup camera and MediaPipe
cap = cv2.VideoCapture(0)
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Define landmark connections for arms
ARM_CONNECTIONS = [
    (mpPose.PoseLandmark.LEFT_SHOULDER, mpPose.PoseLandmark.LEFT_ELBOW),
    (mpPose.PoseLandmark.LEFT_ELBOW, mpPose.PoseLandmark.LEFT_WRIST),
    (mpPose.PoseLandmark.RIGHT_SHOULDER, mpPose.PoseLandmark.RIGHT_ELBOW),
    (mpPose.PoseLandmark.RIGHT_ELBOW, mpPose.PoseLandmark.RIGHT_WRIST),
]

# Initialize pygame sound
pygame.mixer.init()
EXIT_FRAME_SOUND = pygame.mixer.Sound("./comeback.mp3")
MOVE_RIGHT_SOUND = pygame.mixer.Sound("./right.mp3")
MOVE_LEFT_SOUND = pygame.mixer.Sound("./left.mp3")

last_played = {
    "exit": 0,
    "left": 0,
    "right": 0,
    "solenoid_left": 0,
    "solenoid_right": 0,
    "release": 0
}
COOLDOWN = 3
GESTURE_COOLDOWN = 1 # *** adjust cant be too low (solenoid)

def play_sound(sound, key):
    current_time = time.time()
    if current_time - last_played[key] >= COOLDOWN:
        last_played[key] = current_time
        sound.play()
        print(f"Played sound: {key}")

def calculate_angle(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    angle = math.degrees(math.atan2(cy - by, cx - bx) - math.atan2(ay - by, ax - bx))
    return abs(angle)

def send_command(action):
    print(f"COMMAND: {action}")
    if action == "FLIPPER_LEFT":
        GPIO.output(FLIPPER_LEFT_PIN, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(FLIPPER_LEFT_PIN, GPIO.LOW)
    elif action == "FLIPPER_RIGHT":
        GPIO.output(FLIPPER_RIGHT_PIN, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(FLIPPER_RIGHT_PIN, GPIO.LOW)
    elif action == "RELEASE_BALL":
        GPIO.output(STARTER_PIN, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(STARTER_PIN, GPIO.LOW)

try:
    while cap.isOpened():
        success, img = cap.read()
        if not success:
            print("Camera not detected")
            break

        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)
        current_time = time.time()

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # Extract landmarks
            def get_point(landmark):
                return (landmarks[landmark.value].x, landmarks[landmark.value].y)

            left_shoulder = get_point(mpPose.PoseLandmark.LEFT_SHOULDER)
            right_shoulder = get_point(mpPose.PoseLandmark.RIGHT_SHOULDER)
            left_elbow = get_point(mpPose.PoseLandmark.LEFT_ELBOW)
            right_elbow = get_point(mpPose.PoseLandmark.RIGHT_ELBOW)
            left_wrist = get_point(mpPose.PoseLandmark.LEFT_WRIST)
            right_wrist = get_point(mpPose.PoseLandmark.RIGHT_WRIST)

            left_arm_angle = calculate_angle(left_shoulder, left_elbow, left_wrist)
            right_arm_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)

            nose_x = landmarks[mpPose.PoseLandmark.NOSE.value].x

            # Alerts based on position
            if nose_x < 0.1 or nose_x > 0.9:
                play_sound(EXIT_FRAME_SOUND, "exit")
            elif nose_x < 0.3:
                play_sound(MOVE_LEFT_SOUND, "left")
            elif nose_x > 0.7:
                play_sound(MOVE_RIGHT_SOUND, "right")

            # Gesture actions
            if 80 < left_arm_angle < 120 and current_time - last_played["solenoid_left"] > GESTURE_COOLDOWN:
                send_command("FLIPPER_LEFT")
                last_played["solenoid_left"] = current_time

            if 80 < right_arm_angle < 120 and current_time - last_played["solenoid_right"] > GESTURE_COOLDOWN:
                send_command("FLIPPER_RIGHT")
                last_played["solenoid_right"] = current_time

            if left_wrist[1] < left_shoulder[1] and right_wrist[1] < right_shoulder[1]:
                if current_time - last_played["release"] > GESTURE_COOLDOWN:
                    send_command("RELEASE_BALL")
                    last_played["release"] = current_time

            # Draw arm connections
            for connection in ARM_CONNECTIONS:
                start = landmarks[connection[0].value]
                end = landmarks[connection[1].value]
                start_pt = (int(start.x * img.shape[1]), int(start.y * img.shape[0]))
                end_pt = (int(end.x * img.shape[1]), int(end.y * img.shape[0]))
                cv2.line(img, start_pt, end_pt, (0, 255, 0), 3)
                cv2.circle(img, start_pt, 5, (0, 0, 255), -1)
                cv2.circle(img, end_pt, 5, (0, 0, 255), -1)

        else:
            play_sound(EXIT_FRAME_SOUND, "exit")

        cv2.imshow("Arm Tracking with Sound Alerts", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print("Program terminated and GPIO cleaned up.")
