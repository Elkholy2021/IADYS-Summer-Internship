
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Quaternion.h"
#include "control_law.cpp"
#include "calculate_speed.cpp"
#include "path_points.cpp"

std_msgs::Float32 thrust_l, thrust_r, thrust_t;
ros::Time cmdDefined;
convert_gps_to_meter conversion;
control_jellyfishbot jellyfishbot_control_system;
calculate_speads calculate_speads_algorithm;
bool ROS_INFO_FLAG = false;
struct Quaternion {
    double w, x, y, z;
    };

    struct EulerAngles {
        double roll, pitch, yaw;
    };


#define number_points 5
struct demo{
        //array declared inside structure
        double arr[2][number_points];
      };   

demo gpsPoints;
demo meterPoints;
Point pm;
Point Pm;
Speeds Vs;

double delta = 3;


int following_point = 1;
int counter = 0;
int point1 = 0;
double tau_L = 0;
double tau_R = 0;
double tau_M = 0;
int hv = 0;
int counter_gps = 0;
double latitude0;
double longitude0;
int hp = 0;
int xf_10;
int xf_0;
bool change_point = false;
int gps_counter = 0;
int wait_time = 5; //in seconds because gps is 1 hz
double ax, ay, xf, yf;

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
    // geometry_msgs::Vector3 yaw_msg;
    // yaw_msg.x = angles.yaw;
    // jellyfishbot_control_system.yaw_topic.publish(yaw_msg);
    return angles;
    }
void setThrustCmd(float left, float right, float trans) {
    thrust_l.data = left;
    thrust_r.data = right;
    thrust_t.data = trans;
    cmdDefined = ros::Time::now();
}

void RunMyAlgorithm(){
    jellyfishbot_control_system.x = Pm.x;
    jellyfishbot_control_system.y = Pm.y;
    jellyfishbot_control_system.u = Vs.u;
    jellyfishbot_control_system.v = Vs.v;
    jellyfishbot_control_system.r = Vs.r;
    
    
    if (following_point == 1){
        ax = 0;
        ay = 0;
        yf = meterPoints.arr[1][1];
        xf = meterPoints.arr[0][1];
    }
    else{
        ax = meterPoints.arr[0][following_point-1];
        ay = meterPoints.arr[1][following_point-1];
        yf = meterPoints.arr[1][following_point];
        xf = meterPoints.arr[0][following_point];
    }
    
    cout << "ax,ay: "<< ax <<","<< ay << endl;
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
    double threshold = 0.5;

    double u_d= 0.5;
    if (jellyfishbot_control_system.e_psi > 0.1 | jellyfishbot_control_system.e_psi<-0.1){
        u_d = 0.07;
    }
        
    cout << "distance to next point: "<< jellyfishbot_control_system.distance << endl;
    // if (jellyfishbot_control_system.distance < 0.5  & counter > 100 | jellyfishbot_control_system.getting_away){
    //     point1 = point1+1;
    //     counter = 0;
    // }

    //cout << "Counter: " << counter << endl;
    cout << "following_point: "<< following_point<<endl;
    bool check_arrival = jellyfishbot_control_system.check_arrival(threshold);

    if (jellyfishbot_control_system.distance < threshold && counter > 10 ){
        following_point = following_point + 1;
        counter = 0;
        change_point = true;
    }
    
    double v_d= 0;
    jellyfishbot_control_system.u_d_yaw = 5; // desired surge velocity for periods of yaw motion    
    jellyfishbot_control_system.xd = xd ;
    jellyfishbot_control_system.yd = yd;
    jellyfishbot_control_system.u_d = u_d;
    jellyfishbot_control_system.v_d = v_d;
    
    
    //jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target(u_d,v_d,jellyfishbot_control_system.xd,jellyfishbot_control_system.yd);
    jellyfishbot_control_system.obtain__thruster_commands_LOS_Virtual_target_NO_SPEEDS(u_d,v_d,jellyfishbot_control_system.xd,jellyfishbot_control_system.yd);
    geometry_msgs::Vector3 e_psi_msg;
    e_psi_msg.x = jellyfishbot_control_system.e_psi;
    e_psi_msg.y = jellyfishbot_control_system.e_psi_integral_NO_SPEED;
    e_psi_msg.z = jellyfishbot_control_system.e_psi_dot_NO_SPEED;
    jellyfishbot_control_system.e_psi_topic.publish(e_psi_msg);
    tau_L = jellyfishbot_control_system.tau_L;
    tau_R = jellyfishbot_control_system.tau_R;
    tau_M = jellyfishbot_control_system.tau_M;
    geometry_msgs::Vector3 thrusters_forces_msg;
    thrusters_forces_msg.x = tau_L;
    thrusters_forces_msg.y = tau_R;
    thrusters_forces_msg.z = tau_M;
    jellyfishbot_control_system.thrusters_forces_topic.publish(thrusters_forces_msg);
    
    
    tau_L = jellyfishbot_control_system.force_to_pwm(tau_L);
    tau_R = jellyfishbot_control_system.force_to_pwm(tau_R);
    tau_M = jellyfishbot_control_system.force_to_pwm(tau_M);
    geometry_msgs::Vector3 thrusters_rpm_msg;
    thrusters_rpm_msg.x = tau_L;
    thrusters_rpm_msg.y = tau_R;
    thrusters_rpm_msg.z = tau_M;
    jellyfishbot_control_system.thrusters_rpm_topic.publish(thrusters_rpm_msg);

    if (jellyfishbot_control_system.distance < 0.5 && following_point == number_points-1 && counter > 100 && jellyfishbot_control_system.stop == false){
        jellyfishbot_control_system.stop = true;
        
        
    }
    if (jellyfishbot_control_system.stop == true) {
        cout <<"heeey I reached the final point" <<endl;
        tau_L = 1453; //0;
        tau_R = 1453; //0;
        tau_M = 1453; //0;
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
    setThrustCmd(tau_R,tau_L,tau_M);
    // jellyfishbot_control_system.pub_thrust_l.publish(msg_tau_R);
    // jellyfishbot_control_system.pub_thrust_r.publish(msg_tau_L);
    // jellyfishbot_control_system.pub_thrust_tt.publish(msg_tau_M);
    jellyfishbot_control_system.pp_topic.publish(pp_msg);
    jellyfishbot_control_system.vt_topic.publish(vt_msg);

    
}



// ###############################################################################



void odomGPSCallback(const sensor_msgs::NavSatFix &msg)
{
    //TODO
    // setThrustCmd(1, 1, 0);
    if (ROS_INFO_FLAG){
        ROS_INFO("odomGPSCallback");
    }
    counter_gps = counter_gps + 1;
    if (counter_gps < wait_time){
        cout << "waiting ... time left: "<< wait_time - counter_gps +1 << endl;
    }
    if (counter_gps == wait_time){ //initial position
        meterPoints.arr[0][0] = 0;
        meterPoints.arr[1][0] = 0;

        // gpsPoints.arr[0][0] = msg.latitude; 
        // gpsPoints.arr[1][0] =  msg.longitude; 
        
        gpsPoints.arr[0][0] =  43.251740;  //IADYS
        gpsPoints.arr[1][0] =  5.580680; 

        // gpsPoints.arr[0][1] = 43.252561;  //en face building with glasses
        // gpsPoints.arr[1][1] =  5.581275; 
        // gpsPoints.arr[0][1] = 43.252015;  //corner north west building accros street
        // gpsPoints.arr[1][1] =  5.580286; 
        // gpsPoints.arr[0][1] = 43.251811;  //west
        // gpsPoints.arr[1][1] =  5.579929; 
        // gpsPoints.arr[0][1] = 43.251801;  //north east corner of our building
        // gpsPoints.arr[1][1] =  5.580985;
        // gpsPoints.arr[0][1] = 43.251705;  //south east corner of our building
        // gpsPoints.arr[1][1] = 5.580987;
        // gpsPoints.arr[0][1] = 43.251739;  //south west corner of our building
        // gpsPoints.arr[1][1] = 5.580427;
        // gpsPoints.arr[0][1] = 43.251497;  //south west  building
        // gpsPoints.arr[1][1] = 5.579992;
        // gpsPoints.arr[0][1] = 43.251421;  //south west  building2
        // gpsPoints.arr[1][1] = 5.580414;
        // gpsPoints.arr[0][1] = 43.251250;  //south east  building2
        // gpsPoints.arr[1][1] = 5.581190;
        // gpsPoints.arr[0][1] = 43.251994;  //north east  building2
        // gpsPoints.arr[1][1] = 5.581587;
        gpsPoints.arr[0][1] = 43.251831;  //north east  very small
        gpsPoints.arr[1][1] = 5.580704;
        
        
        


        // gpsPoints.arr[0][1] = 43.213793;
        // gpsPoints.arr[1][1] =  5.535779;
        gpsPoints.arr[0][2] = 43.213773;
        gpsPoints.arr[1][2] = 5.535718;
        gpsPoints.arr[0][3] =43.213558;
        gpsPoints.arr[1][3] = 5.535843;

        gpsPoints.arr[0][4] = 43.213756;
        gpsPoints.arr[1][4] =  5.535726;

        conversion.Clatitude1 = gpsPoints.arr[0][0];
        conversion.Clongitude1 = gpsPoints.arr[1][0];
        latitude0 = 43.251740; //IADYS
        longitude0 = 5.580680;
        for (int i = 1; i < number_points; ++i) {
        // for (int i = 0; i < number_points; ++i) {
            //cout << "i: "<< i<< endl;
            conversion.Clatitude2 = gpsPoints.arr[0][i]; 
            conversion.Clongitude2 = gpsPoints.arr[1][i];  
            //pm = conversion.gpsToCoordinatesInMeter(conversion.Clatitude1,conversion.Clongitude1,conversion.Clatitude2,conversion.Clongitude2);
            pm = conversion.gpsToCoordinatesInMeter(latitude0,longitude0,conversion.Clatitude2,conversion.Clongitude2);

            meterPoints.arr[0][i] = pm.x;
            meterPoints.arr[1][i] = pm.y;
        }
    }
    // for (int i = 0; i < 5; i++){
    // cout <<"x,y: "<<meterPoints.arr[0][i] <<","<<meterPoints.arr[1][i]<<endl;
    // }
    
    if (counter_gps == wait_time){
        // latitude0 = msg.latitude;
        // longitude0 = msg.longitude;
        latitude0 = 43.251740; //IADYS
        longitude0 = 5.580680;
    }
    if (counter_gps > wait_time){
    // calculate_speads_algorithm.Clatitude1 = calculate_speads_algorithm.Clatitude2;
    // calculate_speads_algorithm.Clongitude1 = calculate_speads_algorithm.Clongitude2;
    // calculate_speads_algorithm.Clatitude2 = msg.latitude;
    // calculate_speads_algorithm.Clongitude2 = msg.longitude;
    calculate_speads_algorithm.Clatitude1 = 43.251740; //IADYS
    calculate_speads_algorithm.Clongitude1 = 5.580680;
    calculate_speads_algorithm.Clatitude2 = 43.251740;
    calculate_speads_algorithm.Clongitude2 = 5.580680;


    cout << "Yaw angle: " << jellyfishbot_control_system.psi <<endl;
    cout << "Correction: " << calculate_speads_algorithm.heading_correction <<endl;

    cout << "lat1: "<< calculate_speads_algorithm.Clatitude1<< " ,lon1: "<< calculate_speads_algorithm.Clongitude1 << endl;
    cout << "lat2: "<< calculate_speads_algorithm.Clatitude2<< " ,lon2: "<< calculate_speads_algorithm.Clongitude2 << endl;
    double distance = calculate_speads_algorithm.CoordinatesToMeters(calculate_speads_algorithm.Clatitude1 , calculate_speads_algorithm.Clongitude1 , calculate_speads_algorithm.Clatitude2 , calculate_speads_algorithm.Clongitude2 );
    double velocity = calculate_speads_algorithm.calculate_velocity();
    double angle = calculate_speads_algorithm.CoordinatesToAngle(calculate_speads_algorithm.Clatitude1 , calculate_speads_algorithm.Clongitude1 , calculate_speads_algorithm.Clatitude2 , calculate_speads_algorithm.Clongitude2);
    Pm = calculate_speads_algorithm.gpsToCoordinatesInMeter(latitude0 , longitude0 , calculate_speads_algorithm.Clatitude2 , calculate_speads_algorithm.Clongitude2);
    Vs = calculate_speads_algorithm.calculate_speeds(angle);
    cout << "distance: " << distance << endl;
    cout << "velocity: " << velocity << endl;
    cout << "angle: " << angle << endl;
    cout << "x,y: " << Pm.x << "," << Pm.y << endl;
    cout << "u: " << Vs.u << " ,v: " << Vs.v<< " ,r: " << Vs.r << endl;
    geometry_msgs::Quaternion velocities;
    velocities.x = Vs.u;
    velocities.y = Vs.v;
    velocities.z = Vs.r;
    velocities.w = velocity;
    jellyfishbot_control_system.velocities_topic.publish(velocities);

    
    RunMyAlgorithm();
    geometry_msgs::Vector3 distance_to_target_msg;
    distance_to_target_msg.x = jellyfishbot_control_system.distance;
    distance_to_target_msg.y = latitude0;
    distance_to_target_msg.z = longitude0;
    jellyfishbot_control_system.distance_to_target_topic.publish(distance_to_target_msg);

    geometry_msgs::Quaternion path_segment_msg;
    path_segment_msg.x = jellyfishbot_control_system.ax;
    path_segment_msg.y = jellyfishbot_control_system.ay;
    path_segment_msg.z = jellyfishbot_control_system.xf;
    path_segment_msg.w = jellyfishbot_control_system.yf;
    jellyfishbot_control_system.path_segment_topic.publish(path_segment_msg);

    geometry_msgs::Quaternion headings_msg;
    headings_msg.x = jellyfishbot_control_system.psi;
    headings_msg.y = jellyfishbot_control_system.Psi_d;
    headings_msg.z = jellyfishbot_control_system.e_psi;
    headings_msg.w = angle;
    jellyfishbot_control_system.headings_topic.publish(headings_msg);


    }
}

void magHeadingCallback(const geometry_msgs::Quaternion &msg)
{
    //TODO
    if (ROS_INFO_FLAG){
        ROS_INFO("magHeadingCallback");
    }
    hp = hp +1;
    //cout << "Yaw angle: " << endl;
    double xq =  msg.x;
    double yq =  msg.y;
    double zq =  msg.z;
    double wq =  msg.w;
    struct Quaternion q;
    struct EulerAngles EulerAngles;
    q.x = xq;
    q.y = yq;
    q.z = zq;
    q.w = wq;
    EulerAngles = ToEulerAngles(q);
    calculate_speads_algorithm.imuHeading0 = calculate_speads_algorithm.imuHeading;
    //calculate_speads_algorithm.imuHeading = EulerAngles.yaw + calculate_speads_algorithm.heading_correction;
    calculate_speads_algorithm.imuHeading = calculate_speads_algorithm.degreeToRadian(msg.z-90) + calculate_speads_algorithm.heading_correction;
    
    //jellyfishbot_control_system.psi = EulerAngles.yaw;
    jellyfishbot_control_system.psi = calculate_speads_algorithm.imuHeading;
    geometry_msgs::Vector3 yaw_msg;
    yaw_msg.x = calculate_speads_algorithm.degreeToRadian(msg.z-90);
    jellyfishbot_control_system.yaw_topic.publish(yaw_msg);

    //cout << "Yaw angle: " << jellyfishbot_control_system.psi <<endl;

    // if (counter_gps > wait_time){
    //     RunMyAlgorithm();
    // }
}

void headingCorrectionCallback(const std_msgs::Float32 &msg)
{
    //TODO
    if (ROS_INFO_FLAG){

        ROS_INFO("headingCorrectionCallback");
    }
    calculate_speads_algorithm.heading_correction = calculate_speads_algorithm.degreeToRadian(msg.data);

}

// void thrustCmdCallback(const geometry_msgs::Vector3 &cmd)
// {
//     //TODO
//     ROS_INFO("thrustCmdCallback");
// }

/*!
 * \fn int main(int argc, char **argv)
 * \brief Main entry
 * \param[in] argc : System args
 * \param[in] argv : System args
 * \return EXIT_SUCCESS - Success exit
 */
int main(int argc, char **argv) {

    // Init ROS
    ros::init(argc, argv, "testing_node");
    ros::NodeHandle n;

    // Main frequency
    ros::Rate loop_rate(3);

    //pour voir tout les messages
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    //Subs
    ros::Subscriber gps_sub = n.subscribe("positionGPS", 1, odomGPSCallback);                                    // subscribing to the topic "/positionGPS" topic corresponds to the robot position
    ros::Subscriber mag_heading_sub = n.subscribe("eulerIMU", 1, magHeadingCallback);                            // subscribing to the topic "/eulerIMU" topic corresponds to robots Euler angles (x, y and z) and calibration value (w)
    ros::Subscriber heading_correction_sub = n.subscribe("heading_correction", 1000, headingCorrectionCallback); // subscribing to the topic "/heading_correction" topic corresponds to the delta heading correction computed with the historic of robot positions
    //ros::Subscriber thrust_l_sub = n.subscribe("thrustCmd", 1, thrustCmdCallback); // subscribing on thrusturs commands

    //Pubs
    ros::Publisher thrust_l_pub = n.advertise<std_msgs::Float32>("thrust_l0", 1);  // publishing on "thrust_l" topic : left thruster command
    ros::Publisher thrust_r_pub = n.advertise<std_msgs::Float32>("thrust_r0", 1);  // publishing on "thrust_r" topic : right thruster command
    ros::Publisher thrust_t_pub = n.advertise<std_msgs::Float32>("thrust_t0", 1);  // publishing on "thrust_t" topic : central thruster command
    ros::Publisher pp_topic = n.advertise<geometry_msgs::Vector3>("/projected_point_topic", 1000);
    ros::Publisher vt_topic = n.advertise<geometry_msgs::Vector3>("/virtual_target_topic", 1000);
    ros::Publisher velocities_topic = n.advertise<geometry_msgs::Quaternion >("/velocities", 1000);
    ros::Publisher thrusters_forces_topic = n.advertise<geometry_msgs::Vector3 >("/thrusters_forces", 1000);
    ros::Publisher thrusters_rpm_topic = n.advertise<geometry_msgs::Vector3 >("/thrusters_rpm", 1000);
    ros::Publisher distance_to_target_topic = n.advertise<geometry_msgs::Vector3 >("/distance_to_target", 1000);
    ros::Publisher path_segment_topic = n.advertise<geometry_msgs::Quaternion >("/path_segment_topic", 1000);
    ros::Publisher headings_topic = n.advertise<geometry_msgs::Quaternion >("/headings_topic", 1000);
    ros::Publisher yaw_topic = n.advertise<geometry_msgs::Vector3 >("/yaw_topic", 1000);
    ros::Publisher e_psi_topic = n.advertise<geometry_msgs::Vector3 >("/e_psi_topic", 1000);

    

    // jellyfishbot_control_system.pub_thrust_l = thrust_l_pub;
    // jellyfishbot_control_system.pub_thrust_r = thrust_r_pub;
    // jellyfishbot_control_system.pub_thrust_tt = thrust_t_pub;
    jellyfishbot_control_system.pp_topic = pp_topic;
    jellyfishbot_control_system.vt_topic = vt_topic;
    jellyfishbot_control_system.velocities_topic = velocities_topic;
    jellyfishbot_control_system.thrusters_forces_topic = thrusters_forces_topic; 
    jellyfishbot_control_system.thrusters_rpm_topic = thrusters_rpm_topic;
    jellyfishbot_control_system.distance_to_target_topic = distance_to_target_topic;
    jellyfishbot_control_system.path_segment_topic = path_segment_topic;
    jellyfishbot_control_system.headings_topic = headings_topic;
    jellyfishbot_control_system.yaw_topic = yaw_topic;
    jellyfishbot_control_system.e_psi_topic = e_psi_topic;
    thrust_l.data = 0;
    thrust_r.data = 0;
    thrust_t.data = 0;
    cmdDefined = ros::Time::now() - ros::Duration(5);

    // Ros loop
    while (ros::ok()) {

        //Loop action
        if(cmdDefined <= (ros::Time::now() - ros::Duration(2))) {
            thrust_l.data = 0;
            thrust_r.data = 0;
            thrust_t.data = 0;
        }

        thrust_l_pub.publish(thrust_l);
        thrust_r_pub.publish(thrust_r);
        thrust_t_pub.publish(thrust_t);
        if (ROS_INFO_FLAG){
            ROS_INFO("Loop !!");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}