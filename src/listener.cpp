#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

#include "geometry_msgs/Pose.h"
#include "control_law.cpp"
control_jellyfishbot jellyfishbot_control_system;

struct Quaternion {
    double w, x, y, z;
    };

    struct EulerAngles {
        double roll, pitch, yaw;
    };
int following_point = 1;
int counter = 0;
int point1 = 0;
double tau_L = 0;
double tau_R = 0;
double tau_M = 0;



EulerAngles ToEulerAngles(Quaternion q) {
        EulerAngles angles;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }
int hv = 0;
void velocity_callback(const geometry_msgs::Twist& msg)
{   
    hv = hv +1;
    //ROS_INFO("[Listener] I heard speed: [%f]\n",u);
    jellyfishbot_control_system.u = msg.linear.x;
    jellyfishbot_control_system.v = msg.linear.y;
    jellyfishbot_control_system.r = msg.angular.z;
    //cout << "I got in the velocity callback: "<< hv << endl; 
    
}
int hp = 0;
void pose_callback(const geometry_msgs::Pose& msg)
{    
    hp = hp +1;
    double xq =  msg.orientation.x;
    double yq =  msg.orientation.y;
    double zq =  msg.orientation.z;
    double wq =  msg.orientation.w;
    struct Quaternion q;
    struct EulerAngles EulerAngles;
    q.x = xq;
    q.y = yq;
    q.z = zq;
    q.w = wq;
    EulerAngles = ToEulerAngles(q);
    jellyfishbot_control_system.psi = EulerAngles.yaw;
    jellyfishbot_control_system.x = msg.position.x;
    jellyfishbot_control_system.y = msg.position.y;
    //cout << "I got in the pose callback: "<< hp << endl; 
    

    // ROS_INFO("[Listener] I heard poseX: [%f]\n",x);
    // ROS_INFO("[Listener] I heard poseYaw: [%f]\n",yaw);
    //cout << "I got here: "<< h << endl; 
        double ax, ay, xf, yf;
        double delta = 1;
        if (following_point == 1){
            ax = 0;
            ay = 0;
            yf = 3;
            xf = 3;
        }
        
        else if (following_point == 2){
            ax = 3;
            ay = 3;
            yf = 3;
            xf = 10 ;
        }
        else if (following_point == 3){
            ax = 10;
            ay = 3;
            yf = 3.7;
            xf = 10 ;
        }
        else if (following_point == 4){
            ax = 10;
            ay = 3.7;
            yf = 3.7;
            xf = 0 ;
        }
        else if (following_point == 5){
            ax = 0;
            ay = 3.7;
            yf = 4.4;
            xf = 0;
        }  
        else if (following_point == 6){
            ax = 0;
            ay = 4.4;
            yf = 4.4;
            xf = 10;
        } 
        else if (following_point == 7){
            ax = 10;
            ay = 4.4;
            yf = 5.1;
            xf = 10;
        } 
        cout << "xf,yf: "<< xf <<","<< yf << endl;
        jellyfishbot_control_system.ax = ax;
        jellyfishbot_control_system.ay = ay;
        jellyfishbot_control_system.yf = yf;
        jellyfishbot_control_system.xf = xf ;
        jellyfishbot_control_system.delta = delta;
        jellyfishbot_control_system.projected_point();
        double xp = jellyfishbot_control_system.xp;
        double yp = jellyfishbot_control_system.yp;
        //jellyfishbot_control_system.virtual_target(ax,ay,xp,yp,xf,yf,delta,1000000,30); //1000000,30
        jellyfishbot_control_system.virtual_target2(ax,ay,xp,yp,xf,yf,delta,1000000,30); //1000000,30

        double xd = jellyfishbot_control_system.xd;
        double yd = jellyfishbot_control_system.yd;
        if (jellyfishbot_control_system.enable_print){
            //cout <<"ax,ay = " << jellyfishbot_control_system.ax <<","<<jellyfishbot_control_system.ay<<endl;
            //cout << "xp,yp = " << xp<< ","<<yp << endl;
            //cout << "xd,yd = " << xd << ","<< yd<< endl;
            //cout << "xf,yf = "<< xf << "," << yf << endl;
        }
        counter = counter +1;
        
        double u_d= 0.5;
        if (jellyfishbot_control_system.e_psi > 0.1 | jellyfishbot_control_system.e_psi<-0.1){
            u_d = 0.07;
        }
            
        cout << "distance to next point: "<< jellyfishbot_control_system.distance << endl;
        // if (jellyfishbot_control_system.distance < 0.5  & counter > 100 | jellyfishbot_control_system.getting_away){
        //     point1 = point1+1;
        //     counter = 0;
        // }
        double threshold = 0.5;

        cout << "Counter: " << counter << endl;
        if (jellyfishbot_control_system.distance < threshold & following_point ==1 & counter > 100 |  jellyfishbot_control_system.getting_away){
            following_point = 2;
            counter = 0;
        }
        else if (jellyfishbot_control_system.distance < threshold & following_point == 2 & counter > 100 |  jellyfishbot_control_system.getting_away){
            following_point = 3;
            counter = 0;
        }
        else if (jellyfishbot_control_system.distance < threshold & following_point == 3 & counter > 100 |  jellyfishbot_control_system.getting_away){
            following_point = 4;
            counter = 0;
        }
        else if (jellyfishbot_control_system.distance < threshold & following_point == 4 & counter > 100 |  jellyfishbot_control_system.getting_away){
            following_point = 5;
            counter = 0;
        }
        else if (jellyfishbot_control_system.distance < threshold & following_point == 5 & counter > 100 |  jellyfishbot_control_system.getting_away){
            following_point = 6;
            counter = 0;
        }
        else if (jellyfishbot_control_system.distance < threshold & following_point == 6 & counter > 100 |  jellyfishbot_control_system.getting_away){
            following_point = 7;
            counter = 0;
        }

        double v_d= 0;
        jellyfishbot_control_system.u_d_yaw = 5; // desired surge velocity for periods of yaw motion


        
        jellyfishbot_control_system.xd = xd ;
        jellyfishbot_control_system.yd = yd;
        jellyfishbot_control_system.u_d = u_d;
        jellyfishbot_control_system.v_d = v_d;
        cout << "u_d: " <<jellyfishbot_control_system.u_d << ",u: " <<jellyfishbot_control_system.u << endl;
        
        if (! jellyfishbot_control_system.check_arrival(threshold) | 1 <2){
            //cout << "P4 " << endl;
            jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target(u_d,v_d);
            tau_L = jellyfishbot_control_system.tau_L;
            tau_R = jellyfishbot_control_system.tau_R;
            tau_M = jellyfishbot_control_system.tau_M;
            
            if (jellyfishbot_control_system.distance < 0.5 & following_point == 5 & counter > 100 | jellyfishbot_control_system.getting_away){
                jellyfishbot_control_system.stop = true;
                tau_L = 0;
                tau_R = 0;
                tau_M = 0;
                //cout <<"heeey I reached the final point" <<endl;
            }
           
            // pub_thrust_l.publish(msg_tau_R);     
            // pub_thrust_r.publish(msg_tau_L);     
            // pub_thrust_t.publish(msg_tau_M);
            // projected_point_msg = Vector3()
            // projected_point_msg.x = jellyfishbot_control_system.xp
            // projected_point_msg.y = jellyfishbot_control_system.yp
            // projected_point_topic.publish(projected_point_msg)
            // if jellyfishbot_control_system.enable_print:
            //     print("thrusters: {},{},{},    {}, {},{},   {}  {}    {}".format(tau_L,tau_R,tau_M,round(jellyfishbot_control_system.psi_d,3),round(jellyfishbot_control_system.psi,3),round(jellyfishbot_control_system.e_psi,3),jellyfishbot_control_system.movement,round(jellyfishbot_control_system.distance,2),round(jellyfishbot_control_system.u,2)))
           
        }
            std_msgs::Float32 msg_tau_R;
            std_msgs::Float32 msg_tau_L;
            std_msgs::Float32 msg_tau_M;
            geometry_msgs::Vector3 pp_msg;
            geometry_msgs::Vector3 vt_msg;
            msg_tau_L.data = tau_L;
            msg_tau_R.data = tau_R;
            msg_tau_M.data = tau_M;
            pp_msg.x = jellyfishbot_control_system.xp;
            pp_msg.y = jellyfishbot_control_system.yp;

            vt_msg.x = jellyfishbot_control_system.xd;
            vt_msg.y = jellyfishbot_control_system.yd;
            // msg_tau_L.data = 5;
            // msg_tau_R.data = 5;
            // msg_tau_M.data = 0;
    

        

            jellyfishbot_control_system.pub_thrust_l.publish(msg_tau_R);
            jellyfishbot_control_system.pub_thrust_r.publish(msg_tau_L);
            jellyfishbot_control_system.pub_thrust_tt.publish(msg_tau_M);
            jellyfishbot_control_system.pp_topic.publish(pp_msg);
            jellyfishbot_control_system.vt_topic.publish(vt_msg);
	    
       

       


}
    void listener(ros::NodeHandle node,ros::Publisher pub_thrust_l,ros::Publisher pub_thrust_r,ros::Publisher pub_thrust_tt){
        ros::Subscriber sub1 = node.subscribe("/robot_twist_bff", 1000, velocity_callback);
        ros::Subscriber sub2 = node.subscribe("/robot_pose", 1000, pose_callback);
        //ros::Subscriber sub3 = node.subscribe("/robot_twist_bff", 1000, velocity_callback2);

        ros::spin();
    }


int main(int argc, char **argv)
{
    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "listener_node");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;

    string thL = "/thrust_l";
    string thR = "/thrust_r";
    string thT = "/thrust_t";
    // thL = "/thrust_l0";
    // thR = "/thrust_r0";
    // thT = "/thrust_t0";

    ros::Publisher pub_thrust_l = node.advertise<std_msgs::Float32>(thL, 1000);
    ros::Publisher pub_thrust_r = node.advertise<std_msgs::Float32>(thR, 1000);
    ros::Publisher pub_thrust_tt = node.advertise<std_msgs::Float32>(thT, 1000);
    ros::Publisher pp_topic = node.advertise<geometry_msgs::Vector3>("/projected_point_topic", 1000);
    ros::Publisher vt_topic = node.advertise<geometry_msgs::Vector3>("/virtual_target_topic", 1000);
    
    jellyfishbot_control_system.pub_thrust_l = pub_thrust_l;
    jellyfishbot_control_system.pub_thrust_r = pub_thrust_r;
    jellyfishbot_control_system.pub_thrust_tt = pub_thrust_tt;
    jellyfishbot_control_system.pp_topic = pp_topic;
    jellyfishbot_control_system.vt_topic = vt_topic;
    
    
    // ros::Publisher pub_thrust_l = node.advertise<std_msgs::Float32>(thL, 1000);
    // ros::Publisher pub_thrust_r = node.advertise<std_msgs::Float32>(thR, 1000);
    // ros::Publisher pub_thrust_t = node.advertise<std_msgs::Float32>(thT, 1000);

    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.


         //loop_rate.sleep();
        
       
        
        //loop_rate.sleep(); 

//     while (ros::ok()) 
     while (node.ok()){
     listener(node,pub_thrust_l,pub_thrust_r,pub_thrust_tt);
     }    
            
        
        
//    }

    return 0;
}
