import rosbag
from geometry_msgs.msg import Point
import pandas as pd
import sys
import tf
bag_name=str(sys.argv[-1])

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag(bag_name)
topic_robot_pose = '/robot_pose'
topic_robot_twist_bff = '/robot_twist_bff'
topic_sonar = '/br5/distance_sonar'
topic_projected_point = '/projected_point_topic'
topic_virtual_point = '/virtual_target_topic'
column_names_pose = ['x','y','yaw','xp','yp','xd','yd']


df_pose = pd.DataFrame(columns=column_names_pose)

xp = 0
yp = 0
xd = 0
yd =0
for topic, msg, t in bag.read_messages(topics=[topic_robot_pose,topic_projected_point]):
    #print(msg)
    if topic == topic_robot_pose:
        #print(1)
        x = msg.position.x
        y = msg.position.y
        orientation = msg.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        angle_roll = euler[0]
        angle_pitch = euler[1]
        yaw = euler[2]
        
    if topic == topic_projected_point:
        #print(2)
        
        xp = msg.x
        yp = msg.y
        #print(xp,yp)
        #y = msg.y
    if topic == topic_virtual_point:
        #print(2)
        
        xd = msg.x
        yd = msg.y
        #print(xp,yp)
        #y = msg.y

    df_pose = df_pose.append(
        {
        'x': float(x),
        'y':float(y),
        'yaw':float(yaw),
        'xp': float(xp),
        'yp':float (yp),
        'xd':float(xd),
        'yd':float(yd)
        
       },
        ignore_index=True
    )


df_pose.to_csv('pose'+bag_name.split('.')[0]+'.csv')
print("Done!")
