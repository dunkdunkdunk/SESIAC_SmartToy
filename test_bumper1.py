import time
import cv2
import mediapipe as mp
import numpy as np
import pygame
import Jetson.GPIO as GPIO

# 3.3V pin1, IR Sensor pin3, left flipper pin5, right flipper pin7,
# 13 -> 11, 15 -> 13, ..., 27(GND) -> 25
# Start button pin11, Dunk tank pin13, Starter(solenoid ball) pin15, 3.3V pin17, Bumper pin19
END_IR_PIN = 3
FLIPPER_LEFT_PIN = 5
FLIPPER_RIGHT_PIN = 7
START_BUTTON_PIN = 11
DUNK_TANK_PIN = 13
STARTER_PIN = 15
BUMPER_PIN = 19
# use.BOARD (physical pin)* or BCM? 

# ir sensor for end game *active low detect = 0
# dunk tank, bumper for score *active low
GPIO.setmode(GPIO.BOARD)
# commented out END_IR, START_BUTTON

play = False
GPIO.setup(END_IR_PIN, GPIO.IN)
GPIO.setup(START_BUTTON_PIN, GPIO.IN) # press -> high
GPIO.setup(DUNK_TANK_PIN, GPIO.IN)
GPIO.setup(BUMPER_PIN, GPIO.IN)

GPIO.setup(STARTER_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_LEFT_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_RIGHT_PIN, GPIO.OUT)
# output end, dunktank, bumper set low with 0.5s high -> lcd calculate score

# # Wait for button press to start
# print("Press button to start")
# while GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
#     time.sleep(0.2)
# print("Button pressed!")
# ***********************************************

GESTURE_COOLDOWN = 1.0
SOUND_COOLDOWN = 3.0    

pygame.mixer.init()
EXIT_FRAME_SOUND = pygame.mixer.Sound("sound/comeback.wav")
MOVE_RIGHT_SOUND = pygame.mixer.Sound("sound/right.wav")
MOVE_LEFT_SOUND = pygame.mixer.Sound("sound/left.wav")
BEGIN_SOUND = pygame.mixer.Sound("sound/xylophone10.wav")
# BEGIN_SOUND.set_volume(0.8)

pygame.mixer.music.load("sound/bgmusic.wav")
pygame.mixer.music.set_volume(0.8) # vol 0-1
pygame.mixer.music.play(-1) # loop

def play_sound(sound, key):
    current_time = time.time()
    if current_time - last_played_sound[key] >= SOUND_COOLDOWN:
        last_played_sound[key] = current_time
        sound.play()
        # ************************************
        print(f"Played sound: {key}")

def send_command(action):
    print(f"Send command: {action}")
    if action == "FLIPPER_LEFT":
        # GPIO.output(FLIPPER_LEFT_PIN, GPIO.HIGH)
        time.sleep(0.2)
        # GPIO.output(FLIPPER_LEFT_PIN, GPIO.LOW)
    elif action == "FLIPPER_RIGHT":
        # GPIO.output(FLIPPER_RIGHT_PIN, GPIO.HIGH)
        time.sleep(0.2)
        # GPIO.output(FLIPPER_RIGHT_PIN, GPIO.LOW)
    elif action == "RELEASE_BALL":
        # GPIO.output(STARTER_PIN, GPIO.HIGH)
        time.sleep(0.5)
        # GPIO.output(STARTER_PIN, GPIO.LOW)
# ***********************************************

mp_pose = mp.solutions.pose
# pose = mp_pose.Pose(min_detection_confidence=0.5,
#                     min_tracking_confidence=0.5,
#                     static_image_mode=False,
#                     )
pose = mp_pose.Pose()
cap = cv2.VideoCapture(0)
time.sleep(2)

prev_time = 0
last_played = {
    "release": 0,
    "solenoid_left": 0,
    "solenoid_right": 0
}
last_played_sound = {
    "exit": 0,
    "left": 0,
    "right": 0
}

def get_angle(p1, p2, p3):
    a = np.array(p1)
    b = np.array(p2)
    c = np.array(p3)
    cosine_angle = np.dot(a - b, c - b) / (np.linalg.norm(a - b) * np.linalg.norm(c - b))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return np.degrees(angle)

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Camera not detected")
            break

        current_time = time.time()
        
        # fps = 1 / (current_time - prev_time) if prev_time else 0
        # prev_time = current_time
        # print(f"FPS: {fps:.2f}")

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # scaled_frame = cv2.resize(frame_rgb, (0, 0), fx=0.5, fy=0.5)
        result = pose.process(frame_rgb)

        if not result.pose_landmarks:
            if current_time - last_played_sound["exit"] > SOUND_COOLDOWN:
                play_sound(EXIT_FRAME_SOUND, "exit")
                last_played_sound["exit"] = current_time
            continue

        landmarks = result.pose_landmarks.landmark
        nose_x = landmarks[mp_pose.PoseLandmark.NOSE.value].x

        if nose_x < 0.1 or nose_x > 0.9:
            if current_time - last_played_sound["exit"] > SOUND_COOLDOWN:
                play_sound(EXIT_FRAME_SOUND, "exit")
                last_played_sound["exit"] = current_time
            continue
        elif nose_x < 0.3:
            if current_time - last_played_sound["left"] > SOUND_COOLDOWN:
                play_sound(MOVE_LEFT_SOUND, "left")
                last_played_sound["left"] = current_time
            continue
        elif nose_x > 0.7:
            if current_time - last_played_sound["right"] > SOUND_COOLDOWN:
                play_sound(MOVE_RIGHT_SOUND, "right")
                last_played_sound["right"] = current_time
            continue

        def get_point(name):
            lm = landmarks[name.value]
            return [lm.x, lm.y]

        r_shoulder = get_point(mp_pose.PoseLandmark.RIGHT_SHOULDER)
        r_elbow = get_point(mp_pose.PoseLandmark.RIGHT_ELBOW)
        r_wrist = get_point(mp_pose.PoseLandmark.RIGHT_WRIST)

        l_shoulder = get_point(mp_pose.PoseLandmark.LEFT_SHOULDER)
        l_elbow = get_point(mp_pose.PoseLandmark.LEFT_ELBOW)
        l_wrist = get_point(mp_pose.PoseLandmark.LEFT_WRIST)

        right_angle = get_angle(r_shoulder, r_elbow, r_wrist)
        left_angle = get_angle(l_shoulder, l_elbow, l_wrist)

        if not play:
            if l_wrist[1] < l_shoulder[1] and r_wrist[1] < r_shoulder[1]:
                send_command("RELEASE_BALL")
                play = True
                BEGIN_SOUND.play()
                print("Game started!")
                # ******************************
        else:
            # if 80 < left_angle < 120 and current_time - last_played["solenoid_left"] > GESTURE_COOLDOWN:
            if left_angle < 60 and current_time - last_played["solenoid_left"] > GESTURE_COOLDOWN:
                send_command("FLIPPER_LEFT")
                last_played["solenoid_left"] = current_time

            # if 80 < right_angle < 120 and current_time - last_played["solenoid_right"] > GESTURE_COOLDOWN:
            if right_angle < 60 and current_time - last_played["solenoid_right"] > GESTURE_COOLDOWN:
                send_command("FLIPPER_RIGHT")
                last_played["solenoid_right"] = current_time

            # if GPIO.input(END_IR_PIN) == GPIO.HIGH:
            #     print("Game over, waiting for restart...")
            #     play = False

finally:
    cap.release()
    pose.close()
    # GPIO.cleanup()
    print("Program terminated and GPIO cleaned up.")
