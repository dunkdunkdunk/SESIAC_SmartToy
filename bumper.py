import time
import cv2
import mediapipe as mp
import numpy as np
import pygame
import Jetson.GPIO as GPIO

# End IR Sensor pin21 I L, left flipper pin13 L, right flipper pin11 L,
# 13 -> 11, 15 -> 13, ..., 27(GND) -> 25
# Start button pin7 I H, Starter(solenoid ball) pin15 O L
# Bumper input1,2,3 pin37,35,33 I H, Bumper out1,2,3 pin31,19,29 O L
END_IR_PIN = 21
FLIPPER_LEFT_PIN = 23
FLIPPER_RIGHT_PIN = 11
START_BUTTON_PIN = 11
STARTER_PIN = 15
BUMPER_IN1_PIN = 37
BUMPER_IN2_PIN = 35
BUMPER_IN3_PIN = 33
BUMPER_OUT1_PIN = 31
BUMPER_OUT2_PIN = 19
BUMPER_OUT3_PIN = 29
END_SIGNAL_LCD = 38
BUMPER_SIGNAL_LCD = 40
# use.BOARD (physical pin)* or BCM? 

GPIO.setmode(GPIO.BOARD)
# commented out END_IR, START_BUTTON

play = False
GPIO.setup(END_IR_PIN, GPIO.IN)
GPIO.setup(START_BUTTON_PIN, GPIO.IN)
GPIO.setup(BUMPER_IN1_PIN, GPIO.IN)
GPIO.setup(BUMPER_IN2_PIN, GPIO.IN)
GPIO.setup(BUMPER_IN3_PIN, GPIO.IN)

GPIO.setup(BUMPER_OUT1_PIN, GPIO.IN)
GPIO.setup(BUMPER_OUT2_PIN, GPIO.IN)
GPIO.setup(BUMPER_OUT3_PIN, GPIO.IN)
GPIO.setup(STARTER_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_LEFT_PIN, GPIO.OUT)
GPIO.setup(FLIPPER_RIGHT_PIN, GPIO.OUT)
GPIO.setup(END_SIGNAL_LCD, GPIO.OUT)
GPIO.setup(BUMPER_SIGNAL_LCD, GPIO.OUT)
# output end, bumper set low with 0.5s high -> lcd calculate score

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
            if left_angle < 60 and current_time - last_played["solenoid_left"] > GESTURE_COOLDOWN:
                send_command("FLIPPER_LEFT")
                last_played["solenoid_left"] = current_time

            if right_angle < 60 and current_time - last_played["solenoid_right"] > GESTURE_COOLDOWN:
                send_command("FLIPPER_RIGHT")
                last_played["solenoid_right"] = current_time

            # **************************************************************************************************
            # let end signal to lcd, bumper signal to lcd low
            # bumper in High -> end signal High, bumper signal High, Bumper out 
            if GPIO.input(BUMPER_IN1_PIN) == GPIO.HIGH:
                GPIO.output(END_SIGNAL_LCD, GPIO.HIGH)
                GPIO.output(BUMPER_SIGNAL_LCD, GPIO.HIGH)
                GPIO.output(BUMPER_OUT1_PIN, GPIO.LOW)
            # **************************************************************************************************

            # if GPIO.input(END_IR_PIN) == GPIO.HIGH:
            #     print("Game over, waiting for restart...")
            #     play = False

finally:
    cap.release()
    pose.close()
    GPIO.cleanup()
    print("Program terminated and GPIO cleaned up.")