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
topic_velocities = '/velocities'
column_names_pose = ['u','v','r','V']


df_pose = pd.DataFrame(columns=column_names_pose)

xp = 0
yp = 0
xd = 0
yd =0
for topic, msg, t in bag.read_messages(topics=[topic_velocities]):
    #print(msg)
    if topic == topic_velocities:
        #print(1)
        u = msg.x
        v = msg.y
        r = msg.z
        V = msg.w
   
        
    

    df_pose = df_pose.append(
        {
        'u': float(u),
        'v':float(v),
        'r':float(r),
        'V': float(V)
        
       },
        ignore_index=True
    )


df_pose.to_csv('pose'+bag_name.split('.')[0]+'.csv')
print("Done!")
