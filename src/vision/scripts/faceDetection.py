#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import rospkg
import os
from vision.msg import frameInfo


# ===================================
# camera params
# ===================================
camera_matrix = np.array([
    [686.7524367891678,                    0,       315.4776203834513],
    [0,                    687.4812657062831,       250.1768816072397],
    [0,                                    0,                       1]
], dtype=np.float32)

dist_coeffs = np.array([0.121744376569873, -0.233277477793470, 0, 0, 0], dtype=np.float32)

hfov = 0.851720674973233     #[rad] = 48.8[deg]
vfov = 0.499164166070378     #[rad] = 28.6[deg]


def cal_rad(errorX, errorY, width, height):
    return [(errorX / width)  * (hfov / 2), (errorY / height) * (vfov / 2)]

def facedetect():
    # Initialize ROS node
    rospy.init_node('face_detector', anonymous=True)
    pub = rospy.Publisher('face_info', frameInfo, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz to match common webcam framerates
    
    # Get package path and cascade file
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('vision')
    cascade_path = os.path.join(package_path, 'src', 'opencv', 'data', 'haarcascades', 'haarcascade_frontalface_default.xml')
    
    # Check if cascade file exists
    if not os.path.isfile(cascade_path):
        rospy.logerr(f"Cascade file not found at: {cascade_path}")
        return
        
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        rospy.logerr("Failed to load cascade classifier")
        return
    
    try:
        cap = cv2.VideoCapture(0)
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    except:
        rospy.logerr("Video Error")
        return

    font = cv2.FONT_HERSHEY_SIMPLEX
    prev_time = cv2.getTickCount()

    # ===================================
    # calibration added
    # ===================================
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (frame_width, frame_height), 1, (frame_width, frame_height))

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # ===================================
        # calibration added
        # ===================================

        frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        # Initialize message with default values
        msg = frameInfo()
        msg.centerX = frame_width // 2  # Use frame center as default
        msg.centerY = frame_height // 2  # Use frame center as default
        msg.errorX = 0  # Default error when no face detected
        msg.errorY = 0  # Default error when no face detected
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
            # Find the largest face
            largest_face = max(faces, key=lambda face: face[2] * face[3])  # face[2] is w, face[3] is h
            x, y, w, h = largest_face
            # Calculate center of the face
            msg.centerX = x + w//2
            msg.centerY = y + h//2
            # Calculate errors (desired - current)
            error_threshold = 30  # Threshold for considering movement significant
            raw_error_x = (frame_width // 2) - msg.centerX
            raw_error_y = (frame_height // 2) - msg.centerY
            
            # Apply threshold to errors
            msg.errorX = raw_error_x if abs(raw_error_x) > error_threshold else 0
            msg.errorY = raw_error_y if abs(raw_error_y) > error_threshold else 0
            msg.isDetected = True
            
            # Draw rectangle
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            # Draw center point
            cv2.circle(frame, (msg.centerX, msg.centerY), 3, (0,0,255), -1)

        # errorX [pixel -> radian]
        msg.errorX, msg.errorY = cal_rad(msg.errorX, msg.errorY, frame_width, frame_height)


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