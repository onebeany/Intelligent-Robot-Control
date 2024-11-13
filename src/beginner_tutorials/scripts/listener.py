import rospy
from beginner_tutorials.msg import MyMessage

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Curr time %s, num1: %d, num2: %d, num3: %d', data.time, data.nums[0], data.nums[1], data.nums[2])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', MyMessage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
