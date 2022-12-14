#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath> 
#include <stdio.h>
#include<tuple> // for tuple
#include<string>

using namespace std;

#include <iomanip>
#include <vector>
#include <utility>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

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
int main(){
    struct Quaternion q;
    struct EulerAngles EulerAngles;

    q.x = 0.5;
    q.y = 0.6;
    q.z = 0.1;
    q.w = 1;
    EulerAngles = ToEulerAngles( q);
    cout << "roll: "<< EulerAngles.roll << " pitch: " << EulerAngles.pitch << " yaw: "<< EulerAngles.yaw << endl;
    return 0;
}