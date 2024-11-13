import cv2
import numpy as np
import time
from collections import deque

font = cv2.FONT_HERSHEY_SIMPLEX
def facedetect():
    face_cascade = cv2.CascadeClassifier("/home/onebean/catkin_ws/src/vision/src/opencv/data/haarcascades/haarcascade_frontalface_default.xml")
    
    try:
        cap = cv2.VideoCapture(0)
        # Try to use GPU if available
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            face_cascade.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            face_cascade.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    except:
        print("Video Error")
        return

    fps_frames = deque(maxlen=30)
    fps_display_interval = 0.5
    last_fps_display = time.time()
    fps = 0
    frame_count = 0
    last_faces = []  # Store last detected faces

    while True:
        frame_start = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        # Process faces every 3rd frame
        if frame_count % 3 == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            last_faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30))
        
        # Draw faces using last detection
        for (x, y, w, h) in last_faces:
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.putText(frame, 'Faces', (x-5,y-5), font, 0.5, (255,0,0), 2)

        # FPS calculation
        fps_frames.append(time.time() - frame_start)
        if time.time() - last_fps_display > fps_display_interval:
            fps = len(fps_frames) / sum(fps_frames)
            last_fps_display = time.time()

        cv2.putText(frame, f'FPS: {int(fps)}', (10, 30), font, 1, (0, 255, 0), 2)
        cv2.imshow('frame',frame)

        k = cv2.waitKey(1)
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

facedetect()
