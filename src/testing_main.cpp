
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"

std_msgs::Float32 thrust_l, thrust_r, thrust_t;
ros::Time cmdDefined;
void setThrustCmd(float left, float right, float trans) {
    thrust_l.data = left;
    thrust_r.data = right;
    thrust_t.data = trans;
    cmdDefined = ros::Time::now();
}

void odomGPSCallback(const sensor_msgs::NavSatFix &gps)
{
    //TODO
    // setThrustCmd(1, 1, 0);
    ROS_INFO("odomGPSCallback");
}

void magHeadingCallback(const geometry_msgs::Quaternion &head)
{
    //TODO
    ROS_INFO("magHeadingCallback");
}

void headingCorrectionCallback(const std_msgs::Float32 &corr)
{
    //TODO
    ROS_INFO("headingCorrectionCallback");
}

void thrustCmdCallback(const geometry_msgs::Vector3 &cmd)
{
    //TODO
    ROS_INFO("thrustCmdCallback");
}

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
    ros::Subscriber thrust_l_sub = n.subscribe("thrustCmd", 1, thrustCmdCallback); // subscribing on thrusturs commands

    //Pubs
    ros::Publisher thrust_l_pub = n.advertise<std_msgs::Float32>("thrust_l", 1);  // publishing on "thrust_l" topic : left thruster command
    ros::Publisher thrust_r_pub = n.advertise<std_msgs::Float32>("thrust_r", 1);  // publishing on "thrust_r" topic : right thruster command
    ros::Publisher thrust_t_pub = n.advertise<std_msgs::Float32>("thrust_t", 1);  // publishing on "thrust_t" topic : central thruster command
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

        ROS_INFO("Loop !!");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}