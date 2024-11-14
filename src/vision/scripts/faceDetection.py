#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from vision.msg import frameInfo

def facedetect():
    # Initialize ROS node
    rospy.init_node('face_detector', anonymous=True)
    pub = rospy.Publisher('face_info', frameInfo, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz to match common webcam framerates
    
    face_cascade = cv2.CascadeClassifier("/home/onebean/catkin_ws/src/vision/src/opencv/data/haarcascades/haarcascade_frontalface_default.xml")
    
    try:
        cap = cv2.VideoCapture(0)
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    except:
        rospy.logerr("Video Error")
        return

    font = cv2.FONT_HERSHEY_SIMPLEX
    prev_time = cv2.getTickCount()
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Initialize message with default values
        msg = frameInfo()
        msg.centerX = frame_width // 2  # Use frame center as default
        msg.centerY = frame_height // 2  # Use frame center as default
        msg.isDetected = False

        # Calculate FPS
        curr_time = cv2.getTickCount()
        time_elapsed = (curr_time - prev_time) / cv2.getTickFrequency()
        fps = 1 / time_elapsed
        prev_time = curr_time

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5, minSize=(30,30))

        # Update message if face is detected
        if len(faces) > 0:
            # Use the first detected face
            x, y, w, h = faces[0]
            # Calculate center of the face
            msg.centerX = x + w//2
            msg.centerY = y + h//2
            msg.isDetected = True
            
            # Draw rectangle
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            # Draw center point
            cv2.circle(frame, (msg.centerX, msg.centerY), 3, (0,0,255), -1)

        # Display FPS
        cv2.putText(frame, f'FPS: {int(fps)}', (10, 30), font, 1, (0, 255, 0), 2)
        
        # Publish message
        pub.publish(msg)
        
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == 27:  # esc
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        facedetect()
    except rospy.ROSInterruptException:
        pass