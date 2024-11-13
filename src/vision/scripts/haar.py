import cv2
import numpy as np
from datetime import datetime
import os

font = cv2.FONT_HERSHEY_SIMPLEX
def facedetect():
    # Create directory if it doesn't exist
    save_dir = "/home/onebean/catkin_ws/src/vision/src/capturedImages"
    os.makedirs(save_dir, exist_ok=True)

    face_cascade = cv2.CascadeClassifier("/home/onebean/catkin_ws/src/vision/src/opencv/data/haarcascades/haarcascade_frontalface_default.xml")
    
    try:
        cap = cv2.VideoCapture(0)
    except:
        print("Video Error")
        return

    frame_count = 0
    capture_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray,scaleFactor=1.3,minNeighbors=5,minSize=(30,30))

        for (x,y,w,h) in faces:
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.putText(frame,'Faces',(x-5,y-5),font,0.5,(255,0,0),2)

        cv2.imshow('frame',frame)

        # Capture image every 5 frames
        if frame_count % 5 == 0:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(save_dir, f'captured_{timestamp}.jpg')
            cv2.imwrite(filename, frame)
            capture_count += 1
            print(f"Image saved as {filename} (Capture {capture_count}/500)")

            if capture_count >= 500:
                print("Reached 500 captures. Stopping program.")
                break

        if cv2.waitKey(1) == 27: # esc to quit
            break

    cap.release()
    cv2.destroyAllWindows()

facedetect()