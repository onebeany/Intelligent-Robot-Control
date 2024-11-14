#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryCommander(Node):
    def __init__(self):
        super().__init__('joint_trajectory_commander')
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/open_manipulator/trajectory_controller/joint_trajectory',
            10
        )
        
        # 퍼블리셔 준비될 때까지 대기
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.0, 1.0, 0.5]  # 원하는 각도 (라디안)
        point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()  # 이동 시간
        
        traj_msg.points.append(point)
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(traj_msg)
        self.get_logger().info('Published JointTrajectory message.')
        
        # 한 번 퍼블리시 후 타이머 취소
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
