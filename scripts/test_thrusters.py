#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def send_commands_to_thrusters():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    pub_l = rospy.Publisher('/thrust_l', Float32, queue_size=10)
    pub_r = rospy.Publisher('/thrust_r', Float32, queue_size=10)
    pub_t = rospy.Publisher('/thrust_t', Float32, queue_size=10)
    #we need to initialize the node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node 
    rospy.init_node('send_commands_to_thrusters', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(50) # 1hz
    #keep publishing until a Ctrl-C is pressed
    
    while not rospy.is_shutdown():
        pub_l.publish(20)
        pub_r.publish(20)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        send_commands_to_thrusters()
    except rospy.ROSInterruptException:
        pass
