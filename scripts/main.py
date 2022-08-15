#!/usr/bin/env python
from calendar import c

from psutil import NoSuchProcess
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import numpy as np
from tf.transformations import euler_from_quaternion
import control_law
import path_generation
from math import pi, tan, sqrt
from sympy import symbols, Eq, solve
import cmath
from scipy.optimize import fsolve

jellyfishbot_control_system=control_law.control_jellyfishbot()
path = path_generation.path_generation()
path.W = 10
path.L = 10
path.R = 0.70
path.x0 =3
path.y0 = 3
path.alpha = 0



path.x0 = path.x0/2
path.y0 = path.y0/2
#path.enable_overlap()
global path_points
global point
global Npoints

point = 0
path_points = path.generate_points()
Npoints = path.Npoints 
#5print("Number of points is {}".format(Npoints))
import time
global first_time
global counter
counter = 0
first_time = False
global first_time2
first_time2 = False
global following_point
global point1
point1 = 0
following_point = 1

def velocity_callback(message):
    #rospy.loginfo(rospy.get_caller_id() + " u,v,r are %s",[ message.linear.x,message.linear.y,message.angular.z])
    u = message.linear.x
    v = message.linear.y
    r = message.angular.z
    jellyfishbot_control_system.log_current_velocity(u,v,r)
    
command_thrusters = 1

jellyfishbot_control_system.enable_print = True
def pose_callback(message):
    #rospy.loginfo(rospy.get_caller_id() + " u,v,r are %s",[ message.linear.x,message.linear.y,message.angular.z])
    x = message.position.x
    y = message.position.y
    orientation_q=message.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    jellyfishbot_control_system.log_current_heading(yaw)
    jellyfishbot_control_system.log_current_location(x,y)
    ##5print(path_points)
    global point
    global first_time
    global first_time2
    global path_points

    MODE = 2

    global counter
    if not first_time and MODE == 3:
        path_points[0,0] = x
        path_points[1,0] = y
        first_time = True
        
        #path_points=np.array([[0 ,10],[0, 0]])
        if jellyfishbot_control_system.enable_print:
            #5print(path_points)
            pass
        path.plot_path()
    

    
    ############ MODE 1 - One point and then stops ############
    if MODE == 1: 
        
        global first_time2
        xd = 0
        yd = 0
        jellyfishbot_control_system.xf = xd
        jellyfishbot_control_system.yf = yd
        
        u_d= 0.5
        v_d= 0
        jellyfishbot_control_system.u_d_yaw = 5 # desired surge velocity for periods of yaw motion


        threshold = 0.5
        
        jellyfishbot_control_system.xd = xd 
        jellyfishbot_control_system.yd = yd
        jellyfishbot_control_system.u_d = u_d
        jellyfishbot_control_system.v_d = v_d
        if not jellyfishbot_control_system.check_arrival(threshold):# or 1 <2:

            tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target(u_d,v_d)

            pub_thrust_l.publish(tau_R)     
            pub_thrust_r.publish(tau_L)     
            pub_thrust_t.publish(tau_M)
            if jellyfishbot_control_system.enable_print:
                #5print("thrusters: {},{},{},    {}, {},{},   {}  {}      {}".format(tau_L,tau_R,tau_M,round(jellyfishbot_control_system.psi_d,3),round(jellyfishbot_control_system.psi,3),round(jellyfishbot_control_system.e_psi,3),jellyfishbot_control_system.movement,round(jellyfishbot_control_system.distance,2),round(jellyfishbot_control_system.u,2)))
                pass
        else:
            
            jellyfishbot_control_system.stop = True
            tau_L,tau_R,tau_M = 0,0,0
            if jellyfishbot_control_system.enable_print:
                #5print("I reach the point {},{}, haaay".format(xd,yd))
                pass
   


    elif MODE == 2:
        ##5print('555')
        # waypoints
        global first_time2
        global following_point
        global point1
                

        ####
        delta = 1
        #if  first_time2:
        if following_point == 1:
            ax = 0
            ay = 0
            yf = 10 #0
            xf = 0 #-10 
        if following_point == 2:
            ax = 0
            ay = 10
            yf = 10
            xf = 0.7 
        elif following_point == 3:
            ax = 0.7
            ay = 10
            yf = 0
            xf = 0.7 
        elif following_point == 4:
            ax = 0.7
            ay = 0
            yf = 0
            xf = 1.4  
        elif following_point == 5:
            ax = 1.4
            ay = 0
            yf = 10
            xf = 1.4 
        elif following_point == 6:
            ax = 1.4
            ay = 10
            yf = 10
            xf = 2.1
        elif following_point == 7:
            ax = 2.1
            ay = 10
            yf = 0
            xf = 2.1    
        #jellyfishbot_control_system.Yp = Yp 
        jellyfishbot_control_system.ax = ax
        jellyfishbot_control_system.ay = ay
        jellyfishbot_control_system.yf = yf
        jellyfishbot_control_system.xf = xf 
        jellyfishbot_control_system.delta = delta

        
            

        projected_point = jellyfishbot_control_system.projected_point3()

        yp = projected_point[0] #+ ay
        xp = projected_point[1]


        jellyfishbot_control_system.xp = xp
        jellyfishbot_control_system.yp = yp

        yd,xd = jellyfishbot_control_system.virtual_target2()
     
     
        if jellyfishbot_control_system.enable_print:
            #5print("ax,ay = {},{}".format(jellyfishbot_control_system.ax,jellyfishbot_control_system.ay))
            #5print("xp,yp = {},{}".format(xp,yp))
            #5print("xd,yd = {},{}".format(xd,yd))
            #5print("xf,yf = {},{}".format(jellyfishbot_control_system.xf,jellyfishbot_control_system.yf))
            pass
        global counter
            
        counter = counter +1
        
        u_d= 0.5
        cross_track_error = sqrt((xp - x)**2+(yp - y)**2)
        ##5print("Cross track error: {}".format(cross_track_error))
        print("Distance to next point: {}".format(jellyfishbot_control_system.distance))
        if jellyfishbot_control_system.e_psi > 0.1 or jellyfishbot_control_system.e_psi<-0.1:
            u_d = 0.07
            
  
        # if jellyfishbot_control_system.distance < 0.5  and counter > 100 or jellyfishbot_control_system.getting_away:
        #     point1 = point1+1
        #     counter = 0
        ##5print("counter: {}".format(counter))
  
  
        if jellyfishbot_control_system.distance < 0.5 and following_point ==1 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 2
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 2 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 3
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 3 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 4
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 4 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 5
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 5 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 6
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 6 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 7
            counter = 0
        
        v_d= 0
        jellyfishbot_control_system.u_d_yaw = 5 # desired surge velocity for periods of yaw motion


        threshold = 0.5
        jellyfishbot_control_system.xd = xd 
        jellyfishbot_control_system.yd = yd
        jellyfishbot_control_system.u_d = u_d
        jellyfishbot_control_system.v_d = v_d
        if not jellyfishbot_control_system.check_arrival(threshold) or 1 <2:
            
            #tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_V4(xd,yd,u_d,v_d)
            tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target(u_d,v_d)
            #tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_FOSSEN(u_d)

            if jellyfishbot_control_system.distance < 0.5 and following_point == 5 and counter > 100 or jellyfishbot_control_system.getting_away:
                jellyfishbot_control_system.stop = True
                tau_L,tau_R,tau_M = 0,0,0
                #5print("heeey I reached the final point")
            #tau_R = min(tau_R,35)
            #tau_L = min(tau_L,35)
            
            pub_thrust_l.publish(tau_R)     
            pub_thrust_r.publish(tau_L)     
            pub_thrust_t.publish(tau_M)
            projected_point_msg = Vector3()
            projected_point_msg.x = jellyfishbot_control_system.xp
            projected_point_msg.y = jellyfishbot_control_system.yp
            projected_point_topic.publish(projected_point_msg)
            if jellyfishbot_control_system.enable_print:
                #5print("thrusters: {},{},{},    {}, {},{},   {}  {}    {}".format(tau_L,tau_R,tau_M,round(jellyfishbot_control_system.psi_d,3),round(jellyfishbot_control_system.psi,3),round(jellyfishbot_control_system.e_psi,3),jellyfishbot_control_system.movement,round(jellyfishbot_control_system.distance,2),round(jellyfishbot_control_system.u,2)))
                pass

           
        else:
            following_point = following_point +1
            #jellyfishbot_control_system.stop = True
            #tau_L,tau_R,tau_M = 0,0,0
            if jellyfishbot_control_system.enable_print:
                 #5print("I reach the point {},{}, haaay".format(jellyfishbot_control_system.xf,jellyfishbot_control_system.yf))
                pass
        # jellyfishbot_control_system.distance0 = distance0
        # #5print("distance0,distance: {},{}".format(jellyfishbot_control_system.distance0,jellyfishbot_control_system.distance))
        #time.sleep(0.1)
        # info = Vector3()
        # info.x = xd
        # info.y = yd
        # info.z = u_d
        # run_info.publish(info)
    elif MODE == 3:
        ##5print('555')
        # waypoints
        global first_time2
        global following_point
        global point1
        # path_points[0,0] = x
        # path_points[1,0] = y
        first_time = True

        ####
        delta = 0.35
        #if  first_time2:
        if point1 < Npoints:
            ax = path_points[0,point1]
            ay = path_points[1,point1]
            yf = path_points[1,point1+1]
            xf = path_points[0,point1+1] 
        else:
            jellyfishbot_control_system.stop = True
            tau_L,tau_R,tau_M = 0,0,0
            if jellyfishbot_control_system.enable_print:
                #5print("I finished the paaath")
                pass
  
        #jellyfishbot_control_system.Yp = Yp 
        jellyfishbot_control_system.ax = ax
        jellyfishbot_control_system.ay = ay
        jellyfishbot_control_system.yf = yf
        jellyfishbot_control_system.xf = xf 
        jellyfishbot_control_system.delta = delta

        
            

        projected_point = jellyfishbot_control_system.projected_point3()

        yp = projected_point[0] #+ ay
        xp = projected_point[1]


        jellyfishbot_control_system.xp = xp
        jellyfishbot_control_system.yp = yp

        yd,xd = jellyfishbot_control_system.virtual_target2()
     
     
        if jellyfishbot_control_system.enable_print:
            #5print("ax,ay = {},{}".format(jellyfishbot_control_system.ax,jellyfishbot_control_system.ay))
            #5print("xp,yp = {},{}".format(xp,yp))
            #5print("xd,yd = {},{}".format(xd,yd))
            #5print("xf,yf = {},{}".format(jellyfishbot_control_system.xf,jellyfishbot_control_system.yf))
            pass
            
        counter = counter +1
        
        u_d= 0.5
        v_d= 0
        # cross_track_error = sqrt((xp - x)**2+(yp - y)**2)
        # if jellyfishbot_control_system.distance < 1 or cross_track_error > 0.5 or counter < 100 :
        #     u_d = 0.1
        # elif 100 <= counter < 150:
        #     u_d = 0.3
        if jellyfishbot_control_system.e_psi > 0.1 or jellyfishbot_control_system.e_psi<-0.1:
            u_d = 0.1
            
  
        if jellyfishbot_control_system.distance < 0.5  and counter > 100 or jellyfishbot_control_system.getting_away:
            point1 = point1+1
            counter = 0
        
        
        jellyfishbot_control_system.u_d_yaw = 5 # desired surge velocity for periods of yaw motion


        threshold = 0.5
        jellyfishbot_control_system.xd = xd 
        jellyfishbot_control_system.yd = yd
        jellyfishbot_control_system.u_d = u_d
        jellyfishbot_control_system.v_d = v_d
        if not jellyfishbot_control_system.check_arrival(threshold) or 1 <2:
            
            #tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_V4(xd,yd,u_d,v_d)
            tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target(u_d,v_d)
            
            #tau_R = min(tau_R,35)
            #tau_L = min(tau_L,35)
            
            pub_thrust_l.publish(tau_R)     
            pub_thrust_r.publish(tau_L)     
            pub_thrust_t.publish(tau_M)
            projected_point_msg = Vector3()
            projected_point_msg.x = jellyfishbot_control_system.xp
            projected_point_msg.y = jellyfishbot_control_system.yp
            projected_point_topic.publish(projected_point_msg)
            virtual_target_msg = Vector3()
            virtual_target_msg.x = jellyfishbot_control_system.xd
            virtual_target_msg.y = jellyfishbot_control_system.yd
            virtual_target_topic.publish(virtual_target_msg)
            if jellyfishbot_control_system.enable_print:
                #5print("thrusters: {},{},{},    {}, {},{},   {}  {}    {}".format(tau_L,tau_R,tau_M,round(jellyfishbot_control_system.psi_d,3),round(jellyfishbot_control_system.psi,3),round(jellyfishbot_control_system.e_psi,3),jellyfishbot_control_system.movement,round(jellyfishbot_control_system.distance,2),round(jellyfishbot_control_system.u,2)))
                pass
        else:
            following_point = following_point +1
            #jellyfishbot_control_system.stop = True
            #tau_L,tau_R,tau_M = 0,0,0
            if jellyfishbot_control_system.enable_print:
                 #5print("I reach the point {},{}, haaay".format(jellyfishbot_control_system.xf,jellyfishbot_control_system.yf))
                pass
        # jellyfishbot_control_system.distance0 = distance0
        # #5print("distance0,distance: {},{}".format(jellyfishbot_control_system.distance0,jellyfishbot_control_system.distance))
        #time.sleep(0.1)
        # info = Vector3()
        # info.x = xd
        # info.y = yd
        # info.z = u_d
        # run_info.publish(info)*
    elif MODE == 4:
        ##5print('555')
        # waypoints
        global first_time2
        global following_point

                

        ####
        delta = 0.35
        #if  first_time2:
        if following_point == 1:
            ax = 0
            ay = 0
            yf = 0
            xf = 15 
        if following_point == 2:
            ax = 15
            ay = 0
            yf = 0.7
            xf = 15 
        elif following_point == 3:
            ax = 15
            ay = 0.7
            yf = 0.7
            xf = 0
        elif following_point == 4:
            ax = 0
            ay = 0.7
            yf = 1.4
            xf = 0  
        elif following_point == 5:
            ax = 0
            ay = 1.4
            yf = 1.4
            xf = 15  
        #jellyfishbot_control_system.Yp = Yp 
        jellyfishbot_control_system.ax = ax
        jellyfishbot_control_system.ay = ay
        jellyfishbot_control_system.yf = yf
        jellyfishbot_control_system.xf = xf 
        jellyfishbot_control_system.delta = delta

        
            

        projected_point = jellyfishbot_control_system.projected_point3()

        yp = projected_point[0] #+ ay
        xp = projected_point[1]


        jellyfishbot_control_system.xp = xp
        jellyfishbot_control_system.yp = yp

        yd,xd = jellyfishbot_control_system.virtual_target2()
     
     
        if jellyfishbot_control_system.enable_print:
            #5print("ax,ay = {},{}".format(jellyfishbot_control_system.ax,jellyfishbot_control_system.ay))
            #5print("xp,yp = {},{}".format(xp,yp))
            #5print("xd,yd = {},{}".format(xd,yd))
            #5print("xf,yf = {},{}".format(jellyfishbot_control_system.xf,jellyfishbot_control_system.yf))
            pass
        global counter
            
        counter = counter +1
        
        u_d= 0.5

        cross_track_error = sqrt((xp - x)**2+(yp - y)**2)
        # if jellyfishbot_control_system.distance < 1 or cross_track_error > 0.5 or counter < 100 :
        #     u_d = 0.1
        # elif 100 <= counter < 150:
        #     u_d = 0.5

        if jellyfishbot_control_system.e_psi > 0.1 or jellyfishbot_control_system.e_psi<-0.1:
            u_d = 0.1
  
  
        if jellyfishbot_control_system.distance < 0.5 and following_point ==1 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 2
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 2 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 3
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 3 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 4
            counter = 0
        elif jellyfishbot_control_system.distance < 0.5 and following_point == 4 and counter > 100 or jellyfishbot_control_system.getting_away:
            following_point = 5
            counter = 0
        
        v_d= 0
        jellyfishbot_control_system.u_d_yaw = 5 # desired surge velocity for periods of yaw motion


        threshold = 0.5
        jellyfishbot_control_system.xd = xd 
        jellyfishbot_control_system.yd = yd
        jellyfishbot_control_system.u_d = u_d
        jellyfishbot_control_system.v_d = v_d
        if not jellyfishbot_control_system.check_arrival(threshold) or 1 <2:
            
            tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target(u_d)
            #tau_L,tau_R,tau_M = jellyfishbot_control_system.obtain__thruster_commands_LOS_FOSSEN(u_d)

            if jellyfishbot_control_system.distance < 0.5 and following_point == 5 and counter > 100 or jellyfishbot_control_system.getting_away:
                jellyfishbot_control_system.stop = True
                tau_L,tau_R,tau_M = 0,0,0
                #5print("heeey I reached the final point")
            #tau_R = min(tau_R,35)
            #tau_L = min(tau_L,35)
            
            pub_thrust_l.publish(tau_R)     
            pub_thrust_r.publish(tau_L)     
            pub_thrust_t.publish(tau_M)
            projected_point_msg = Vector3()
            projected_point_msg.x = jellyfishbot_control_system.xp
            projected_point_msg.y = jellyfishbot_control_system.yp
            projected_point_topic.publish(projected_point_msg)
            virtual_target_msg = Vector3()
            virtual_target_msg.x = jellyfishbot_control_system.xd
            virtual_target_msg.y = jellyfishbot_control_system.yd
            virtual_target_topic.publish(virtual_target_msg)
            if jellyfishbot_control_system.enable_print:
                #5print("thrusters: {},{},{},    {}, {},{},   {}  {}    {}".format(tau_L,tau_R,tau_M,round(jellyfishbot_control_system.psi_d,3),round(jellyfishbot_control_system.psi,3),round(jellyfishbot_control_system.e_psi,3),jellyfishbot_control_system.movement,round(jellyfishbot_control_system.distance,2),round(jellyfishbot_control_system.u,2)))
                pass

           
        else:
            following_point = following_point +1
            #jellyfishbot_control_system.stop = True
            #tau_L,tau_R,tau_M = 0,0,0
            if jellyfishbot_control_system.enable_print:
                 #5print("I reach the point {},{}, haaay".format(jellyfishbot_control_system.xf,jellyfishbot_control_system.yf))
                pass
        # jellyfishbot_control_system.distance0 = distance0
        # #5print("distance0,distance: {},{}".format(jellyfishbot_control_system.distance0,jellyfishbot_control_system.distance))
        #time.sleep(0.1)
        # info = Vector3()
        # info.x = xd
        # info.y = yd
        # info.z = u_d
        # run_info.publish(info)
        
def listener():
    
    # rospy.init_node('listener', anonymous=False)
    # rate = rospy.Rate(10)

    

    rospy.Subscriber("/robot_twist_bff", Twist, velocity_callback)
    rospy.Subscriber("/robot_pose", Pose, pose_callback)
    # rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    if command_thrusters ==1:
        pub_thrust_l = rospy.Publisher('/thrust_l', Float32, queue_size=10)
        pub_thrust_r = rospy.Publisher('/thrust_r', Float32, queue_size=10)
        pub_thrust_t = rospy.Publisher('/thrust_t', Float32, queue_size=10)
        # rospy.Timer(rospy.Duration(1.0/10.0), pub_thrust_l)
        # rospy.Timer(rospy.Duration(1.0/10.0), pub_thrust_r)
        # rospy.Timer(rospy.Duration(1.0/10.0), pub_thrust_t)
    else:
        pub_thrust_l = rospy.Publisher('/thrust_l_off', Float32, queue_size=10)
        pub_thrust_r = rospy.Publisher('/thrust_r_off', Float32, queue_size=10)
        pub_thrust_t = rospy.Publisher('/thrust_t_off', Float32, queue_size=10) 
    projected_point_topic = rospy.Publisher('/projected_point_topic', Vector3, queue_size=10)
    virtual_target_topic = rospy.Publisher('/virtual_target_topic', Vector3, queue_size=10)

    run_info = rospy.Publisher('/run_info', Vector3, queue_size=10)
    
    listener()
