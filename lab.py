import cv2
import mediapipe as mp
import math

# Initialize camera
cap = cv2.VideoCapture(0)

# Initialize Mediapipe Pose
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Define connections for the arms only
ARM_CONNECTIONS = [
    (mpPose.PoseLandmark.LEFT_SHOULDER, mpPose.PoseLandmark.LEFT_ELBOW),
    (mpPose.PoseLandmark.LEFT_ELBOW, mpPose.PoseLandmark.LEFT_WRIST),
    (mpPose.PoseLandmark.RIGHT_SHOULDER, mpPose.PoseLandmark.RIGHT_ELBOW),
    (mpPose.PoseLandmark.RIGHT_ELBOW, mpPose.PoseLandmark.RIGHT_WRIST),
]

def calculate_angle(a, b, c):
    """Calculate angle between three points (shoulder, elbow, wrist)"""
    ax, ay = a
    bx, by = b
    cx, cy = c
    
    angle = math.degrees(math.atan2(cy - by, cx - bx) - math.atan2(ay - by, ax - bx))
    return abs(angle)

while True:
    success, img = cap.read()
    if not success:
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # Get arm landmark coordinates
        left_shoulder = (landmarks[mpPose.PoseLandmark.LEFT_SHOULDER.value].x, 
                         landmarks[mpPose.PoseLandmark.LEFT_SHOULDER.value].y)
        left_elbow = (landmarks[mpPose.PoseLandmark.LEFT_ELBOW.value].x, 
                      landmarks[mpPose.PoseLandmark.LEFT_ELBOW.value].y)
        left_wrist = (landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].x, 
                      landmarks[mpPose.PoseLandmark.LEFT_WRIST.value].y)

        right_shoulder = (landmarks[mpPose.PoseLandmark.RIGHT_SHOULDER.value].x, 
                          landmarks[mpPose.PoseLandmark.RIGHT_SHOULDER.value].y)
        right_elbow = (landmarks[mpPose.PoseLandmark.RIGHT_ELBOW.value].x, 
                       landmarks[mpPose.PoseLandmark.RIGHT_ELBOW.value].y)
        right_wrist = (landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].x, 
                       landmarks[mpPose.PoseLandmark.RIGHT_WRIST.value].y)

        # Calculate angles
        left_arm_angle = calculate_angle(left_shoulder, left_elbow, left_wrist)
        right_arm_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)

        # Print based on angles
        if 60 < left_arm_angle < 120:
            print("Left")

        if 60 < right_arm_angle < 120:
            print("Right")

        # Draw only arm landmarks and connections
        for connection in ARM_CONNECTIONS:
            start = landmarks[connection[0].value]
            end = landmarks[connection[1].value]

            start_point = (int(start.x * img.shape[1]), int(start.y * img.shape[0]))
            end_point = (int(end.x * img.shape[1]), int(end.y * img.shape[0]))

            cv2.line(img, start_point, end_point, (0, 255, 0), 3)  # Green lines
            cv2.circle(img, start_point, 5, (0, 0, 255), -1)  # Red dots
            cv2.circle(img, end_point, 5, (0, 0, 255), -1)

    cv2.imshow("Arm Tracking", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
