import cv2
import numpy as np
import math

def calculate_angle(a, b, c):
    # Angle at point b
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return np.degrees(angle)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Optional: flip frame to mirror user
    frame = cv2.flip(frame, 1)
    output = frame.copy()

    # Convert to HSV and create skin mask (can adjust this!)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_skin = np.array([0, 20, 70], dtype=np.uint8)
    upper_skin = np.array([20, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_skin, upper_skin)

    # Get contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]  # Assume 2 biggest are arms

    for cnt in contours:
        hull = cv2.convexHull(cnt)
        cv2.drawContours(output, [hull], -1, (0, 255, 0), 2)

        # Approximate 3 key points: shoulder, elbow, wrist
        # In real application, use trained model or define fixed positions
        if len(cnt) >= 3:
            pts = cnt[:, 0, :]
            pts = pts[np.argsort(pts[:, 1])]  # sort by Y (top to bottom)
            shoulder = pts[0]
            elbow = pts[len(pts)//2]
            wrist = pts[-1]

            # Draw points
            for pt in [shoulder, elbow, wrist]:
                cv2.circle(output, tuple(pt), 6, (0, 0, 255), -1)

            # Compute angle
            angle = calculate_angle(shoulder, elbow, wrist)
            text = "Curl" if angle < 90 else "Extend"
            cv2.putText(output, f"{text} ({int(angle)}°)", tuple(elbow), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Arm Curl Detection", output)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
