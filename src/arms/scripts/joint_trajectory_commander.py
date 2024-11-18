#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryCommander:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('joint_trajectory_commander', anonymous=True)
        
        # 퍼블리셔 생성
        self.publisher_ = rospy.Publisher(
            '/open_manipulator/trajectory_controller/command',
            JointTrajectory,
            queue_size=10
        )
        
        # 퍼블리셔 준비될 때까지 대기
        rospy.sleep(1)
        
        # 트래젝터리 퍼블리시
        self.publish_trajectory()
        
    def publish_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # 트래젝터리 포인트 설정
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, -1.5, 1.5]  # 원하는 각도 (라디안)
<<<<<<< HEAD
        point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()  # 이동 시간
=======
        point.time_from_start = rospy.Duration(2.0)  # 이동 시간
>>>>>>> 947eccdb2835b9230dac6b3c103729d03044f2cd
        
        # 트래젝터리에 포인트 추가
        traj_msg.points.append(point)
        traj_msg.header.stamp = rospy.Time.now()
        
        # 메시지 퍼블리시
        self.publisher_.publish(traj_msg)
        rospy.loginfo('Published JointTrajectory message.')
        
def main():
    try:
        commander = JointTrajectoryCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
