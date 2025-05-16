import time
import cv2
import mediapipe as mp
import numpy as np
import pygame
import Jetson.GPIO as GPIO
import serial

# End IR Sensor pin11 I L, left flipper pin21 O L, right flipper pin19 O L,
# Start button pin7 I H, Starter(solenoid ball) pin23 O L
END_IR_PIN = 11
START_BUTTON_PIN = 7
END_SIGNAL_LCD = 38
# STARTER_PIN = 23              pin8 ard
# FLIPPER_LEFT_PIN = 21         pin9 ard
# FLIPPER_RIGHT_PIN = 19        pin10 ard


GPIO.setmode(GPIO.BOARD)
# commented out END_IR, START_BUTTON

play = False
GPIO.setup(END_IR_PIN, GPIO.IN)
GPIO.setup(START_BUTTON_PIN, GPIO.IN)
GPIO.setup(END_SIGNAL_LCD, GPIO.OUT)
# GPIO.setup(STARTER_PIN, GPIO.OUT)
# GPIO.setup(FLIPPER_LEFT_PIN, GPIO.OUT)
# GPIO.setup(FLIPPER_RIGHT_PIN, GPIO.OUT)

pygame.mixer.init()
EXIT_FRAME_SOUND = pygame.mixer.Sound("sound/Away.wav")
MOVE_RIGHT_SOUND = pygame.mixer.Sound("sound/Move Right.wav")
MOVE_LEFT_SOUND = pygame.mixer.Sound("sound/Move Left.wav")
BUTTON_SOUND = pygame.mixer.Sound("sound/xylophone10.wav")
BEGIN_SOUND = pygame.mixer.Sound("sound/Game Start.wav")
TUTORIAL_SOUND = pygame.mixer.Sound("sound/Tutorial.wav")
END_SOUND = pygame.mixer.Sound("sound/Game Over.wav")
PLAY_AGAIN_SOUND = pygame.mixer.Sound("Take ball back.wav")

arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# time.sleep(1)

# Wait for button press to start
print("Press button to start")
while GPIO.input(START_BUTTON_PIN) == GPIO.LOW:
    time.sleep(0.2)
print("Button pressed!")

GESTURE_COOLDOWN = 1.0
SOUND_COOLDOWN = 3.0    

pygame.mixer.init()
EXIT_FRAME_SOUND = pygame.mixer.Sound("sound/Away.wav")
MOVE_RIGHT_SOUND = pygame.mixer.Sound("sound/Move Right.wav")
MOVE_LEFT_SOUND = pygame.mixer.Sound("sound/Move Left.wav")
BUTTON_SOUND = pygame.mixer.Sound("sound/xylophone10.wav")
BEGIN_SOUND = pygame.mixer.Sound("sound/Game Start.wav")
TUTORIAL_SOUND = pygame.mixer.Sound("sound/Tutorial.wav")
END_SOUND = pygame.mixer.Sound("sound/Game Over.wav")
PLAY_AGAIN_SOUND = pygame.mixer.Sound("Take ball back.wav")

# BEGIN_SOUND.set_volume(0.8)

pygame.mixer.music.load("sound/bgmusic.wav")
pygame.mixer.music.set_volume(0.8) # vol 0-1
pygame.mixer.music.play(-1) # loop

def get_point(name):
    lm = landmarks[name.value]
    return [lm.x, lm.y]

def play_sound(sound, key):
    current_time = time.time()
    if current_time - last_played_sound[key] >= SOUND_COOLDOWN:
        last_played_sound[key] = current_time
        sound.play()
        # ************************************
        print(f"Played sound: {key}")

def send_command(action):
    # ***************************************
    print(f"Send command: {action}")
    # if action == "FLIPPER_LEFT":
        # GPIO.output(FLIPPER_LEFT_PIN, GPIO.HIGH)
        # time.sleep(0.2)
        # GPIO.output(FLIPPER_LEFT_PIN, GPIO.LOW)
    # elif action == "FLIPPER_RIGHT":
        # GPIO.output(FLIPPER_RIGHT_PIN, GPIO.HIGH)
        # time.sleep(0.2)
        # GPIO.output(FLIPPER_RIGHT_PIN, GPIO.LOW)
    # elif action == "RELEASE_BALL":
        # GPIO.output(STARTER_PIN, GPIO.HIGH)
        # time.sleep(0.5)
        # GPIO.output(STARTER_PIN, GPIO.LOW)
def get_angle(p1, p2, p3):
    a = np.array(p1)
    b = np.array(p2)
    c = np.array(p3)
    cosine_angle = np.dot(a - b, c - b) / (np.linalg.norm(a - b) * np.linalg.norm(c - b))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return np.degrees(angle)

def handle_flipper_gestures(l_a, r_a, prev, curr, cd):
    if l_a < 60 and curr - prev["solenoid_left"] > cd:
        print("Flipper Left Triggered")
        arduino.write("l".encode())
        prev["solenoid_left"] = curr

    if r_a < 60 and curr - prev["solenoid_right"] > cd:
        print("Flipper Right Triggered")
        arduino.write("r".encode())
        prev["solenoid_right"] = curr

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

r_shoulder = get_point(mp_pose.PoseLandmark.RIGHT_SHOULDER)
r_elbow = get_point(mp_pose.PoseLandmark.RIGHT_ELBOW)
r_wrist = get_point(mp_pose.PoseLandmark.RIGHT_WRIST)

l_shoulder = get_point(mp_pose.PoseLandmark.LEFT_SHOULDER)
l_elbow = get_point(mp_pose.PoseLandmark.LEFT_ELBOW)
l_wrist = get_point(mp_pose.PoseLandmark.LEFT_WRIST)

right_angle = get_angle(r_shoulder, r_elbow, r_wrist)
left_angle = get_angle(l_shoulder, l_elbow, l_wrist)

tutorial_started = False
# tutorial_completed = False

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
        # tutorial ***************************
        else:
            if not tutorial_started:
                tutorial_started = True
                pygame.mixer.music.stop()
                TUTORIAL_SOUND.play()
                print("Player centered — starting tutorial")
                if left_angle < 60:
                    print("Tutorial: Left flipper test")
                    print("FLIPPER_LEFT")
                    arduino.write("l".encode())
                if right_angle < 60:
                    print("Tutorial: Right flipper test")
                    print("FLIPPER_RIGHT")
                    arduino.write("r".encode())

                if l_wrist[1] < l_shoulder[1] and r_wrist[1] < r_shoulder[1]:
                    # tutorial_completed = True
                    TUTORIAL_SOUND.stop()
                    pygame.mixer.music.play(-1)
                    print("Tutorial complete — game starting")
                    print("RELEASE_BALL")
                    arduino.write("s".encode())
                    play = True
                    print("Game started!")
                continue

        # if tutorial_started and not tutorial_completed:
        #     if left_angle < 60:
        #         print("Tutorial: Left flipper test")
        #         print("FLIPPER_LEFT")
        #         arduino.write("l".encode())
        #     if right_angle < 60:
        #         print("Tutorial: Right flipper test")
        #         print("FLIPPER_RIGHT")
        #         arduino.write("r".encode())

        #     if l_wrist[1] < l_shoulder[1] and r_wrist[1] < r_shoulder[1]:
        #         tutorial_completed = True
        #         TUTORIAL_SOUND.stop()
        #         pygame.mixer.music.play(-1)
        #         print("Tutorial complete — game starting")
        #         print("RELEASE_BALL")
        #         arduino.write("s".encode())
        #         play = True
        #         print("Game started!")
        #     continue
        if not play:
            if l_wrist[1] < l_shoulder[1] and r_wrist[1] < r_shoulder[1]:
                play = True
                BEGIN_SOUND.play()
                print("RELEASE_BALL")
                arduino.write("s".encode())
                print("Game started!")
        else:
            handle_flipper_gestures(left_angle, right_angle, last_played, current_time, GESTURE_COOLDOWN)
            play = False
            
            # **************************************************************************************************
            
            # *************************************************************************************************

            # if GPIO.input(BUMPER_IN1_PIN) or GPIO.input(BUMPER_IN2_PIN) or GPIO.input(BUMPER_IN3_PIN) == GPIO.HIGH
                
            # if GPIO.input(END_IR_PIN) == GPIO.HIGH:
            #     print("Game over, waiting for restart...")
            #     play = False
            #     END_SOUND.play()

finally:
    arduino.close
    cap.release()
    pose.close()
    GPIO.cleanup()
    print("Program terminated and GPIO cleaned up.")