import rospy
import random
from beginner_tutorials.msg import MyMessage

def talker():
    pub = rospy.Publisher('chatter', MyMessage, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = MyMessage()
        message.nums = [random.randint(0,100) for _ in range(3)]
        message.time = str(rospy.get_time())
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
