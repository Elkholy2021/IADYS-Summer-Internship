#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

def telop_callback(message):
    if message.linear.x > 0:
        thrust_l = 10
        thrust_r = 10
    elif message.linear.x < 0:
        thrust_l = -10
        thrust_r = -10
    elif message.angular.z > 0:
        thrust_l = 0
        thrust_r = 10
    elif message.angular.z < 0:
        thrust_l = 10
        thrust_r = 0
    else:
        thrust_l = 0
        thrust_r = 0
    pub_thrust_l.publish(thrust_l)
    pub_thrust_r.publish(thrust_r)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", message.data)

def listener():

  
    rospy.init_node('combine_thrusts', anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel", Twist, telop_callback)
    rospy.spin()

if __name__ == '__main__':
    pub_thrust_l = rospy.Publisher('/thrust_l', Float32, queue_size=10)
    pub_thrust_r = rospy.Publisher('/thrust_r', Float32, queue_size=10)
    listener()
