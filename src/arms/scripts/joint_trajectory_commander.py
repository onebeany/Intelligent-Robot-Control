#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node('joint_trajectory_commander')

    # 퍼블리셔 생성
    pub = rospy.Publisher('/open_manipulator/goal_joint_space_path', JointTrajectory, queue_size=10)

    # 퍼블리셔가 준비될 때까지 대기
    rospy.sleep(1)

    # JointTrajectory 메시지 생성
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    
    # 목표 위치 설정
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0, -1.5, 1.5]  # 원하는 조인트 각도 (라디안 단위)
    point.time_from_start = rospy.Duration(2.0)  # 이동 시간

    traj_msg.points.append(point)
    traj_msg.header.stamp = rospy.Time.now()

    # 메시지 퍼블리시
    pub.publish(traj_msg)

    rospy.loginfo("Published JointTrajectory message.")
    rospy.sleep(2)  # 이동 시간이 끝날 때까지 대기

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
