import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # 0 = default webcam

def get_background(cap, num_frames=30):
    frames = []
    for _ in range(num_frames):
        ret, frame = cap.read()
        if ret:
            frames.append(frame)
    return np.median(frames, axis=0).astype(np.uint8)

def detect_block(frame, background, threshold=30):
    diff = cv2.absdiff(frame, background)
    gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_diff, threshold, 255, cv2.THRESH_BINARY)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    M = cv2.moments(mask, binaryImage=True)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy

background = get_background(cap, num_frames=30)
ret, frame = cap.read()  # ret = True/False (did it succeed), frame = the image


if ret:
    cv2.imwrite('captured_frame.jpg', frame)  # save it to check
    print('Frame captured, shape:', frame.shape)
else:
    print('Failed to capture frame')
cx, cy = detect_block(frame, background, threshold=30)
cap.release()
