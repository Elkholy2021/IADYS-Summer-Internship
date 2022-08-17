import rosbag
from geometry_msgs.msg import Point
import pandas as pd
import sys
import tf
bag_name=str(sys.argv[-1])

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag(bag_name)
topic_GPS = '/positionGPS'
topic_robot_twist_bff = '/robot_twist_bff'
topic_sonar = '/br5/distance_sonar'
topic_projected_point = '/projected_point_topic'
topic_virtual_point = '/virtual_target_topic'
column_names_pose = ['x','y','yaw']


df_pose = pd.DataFrame(columns=column_names_pose)

xp = 0
yp = 0
xd = 0
yd =0
for topic, msg, t in bag.read_messages(topics=[topic_GPS]):
    #print(msg)
    if topic == topic_GPS:
        #print(1)
        x = msg.latitude
        y = msg.longitude
   
        
    

    df_pose = df_pose.append(
        {
        'x': float(x),
        'y':float(y),
        'yaw':float(0)
        
       },
        ignore_index=True
    )


df_pose.to_csv('pose'+bag_name.split('.')[0]+'.csv')
print("Done!")
