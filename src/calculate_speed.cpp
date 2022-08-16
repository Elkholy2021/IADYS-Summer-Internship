#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
#include "ros/ros.h"

using namespace std;


class calculate_speads{
    public:

        double timeStamp0;
        double distance = 0;
        double velocity = 0;
        double timeStamp;
        double PI = 3.14159265;
        struct Point
        {
        double x;
        double y;
        double z;
        };
        
       
        double latitude1 = 100;
        double longitude1 = 0;
        double latitude2 = 0;
        double longitude2 = 0;
        double imuHeading = 0;
        double imuHeading0 = 0;
        double gpsHeading = 0;
        double latitudeR1;
        double longitudeR1;
        double latitudeR2;
        double longitudeR2;

    double degreeToRadian(const double degree)
    {
        return (degree * PI / 180);
    };

    double radianToDegree(const double radian)
    {
        return (radian * 180 / PI);
    };



    double CoordinatesToMeters()
    {
        
        latitudeR1 = degreeToRadian(latitude1);
        longitudeR1 = degreeToRadian(longitude1);
        latitudeR2 = degreeToRadian(latitude2);
        longitudeR2 = degreeToRadian(longitude2);

        double earthDiameterMeters = 12756000; 

        auto x = sin((latitudeR2 - latitudeR1) / 2), y = sin((longitudeR2 - longitudeR1) / 2);
        if (true) {
            distance = earthDiameterMeters * asin(sqrt((x * x) + (cos(latitudeR1) * cos(latitudeR2) * y * y)));
            return distance;
        
        }
        else{
            auto value = (x * x) + (cos(latitudeR1) * cos(latitudeR2) * y * y);
            distance = earthDiameterMeters * atan2(sqrt(value), sqrt(1 - value));
            return distance;
            }

    }



    double CoordinatesToAngle()
    {
        const auto longitudeDifference = degreeToRadian(longitude2 - longitude1);
        latitudeR1 = degreeToRadian(latitude1);
        latitudeR2 = degreeToRadian(latitude2);



        const auto x = (cos(latitudeR1) * sin(latitudeR2)) -
                        (sin(latitudeR1) * cos(latitudeR2) * cos(longitudeDifference));
        const auto y = sin(longitudeDifference) * cos(latitudeR2);



        const auto degree = radianToDegree(atan2(y, x));
        //return (degree >= 0) ? degree : (degree + 360);
        gpsHeading = degreeToRadian(degree);
        return gpsHeading;
    }


    Point gpsToCoordinatesInMeter()
    {
    Point p;



    auto angle = CoordinatesToAngle();
    // cout << "Angle =  " << angle << endl;

    auto meters = CoordinatesToMeters();
    // cout << "Meters = " << meters << endl;



    // auto coordinate = CoordinateToCoordinate(latitude1, longitude1, angle, meters);
    // cout << "Destination = (" << coordinate.first << "," << coordinate.second << ")\n";



    p.x = meters * cos(degreeToRadian(angle));
    p.y = meters * sin(degreeToRadian(angle));
    return p;
    }
    double calculate_velocity(){

        velocity = distance / (timeStamp - timeStamp0);
        return velocity;

    }
    // Speeds calculate_speeds(){

    //     Speeds Vs;
    //     Vs.u = velocity*cos(imuHeading-gpsHeading);
    //     Vs.v = velocity*sin(imuHeading-gpsHeading);
    //     Vs.r = (imuHeading - imuHeading0)/(timeStamp - timeStamp0);
    //     return Vs;


    //}
};

// calculate_speads calculate_speads_algorithm;
// int main(){


// calculate_speads_algorithm.latitude1 = 43.213552;
// calculate_speads_algorithm.longitude1 = 5.536321;
// calculate_speads_algorithm. latitude2 = 43.213086 ;//43.213897;
// calculate_speads_algorithm.longitude2 = 5.536408; //5.536085;
// calculate_speads_algorithm.timeStamp = 18.5782;
// calculate_speads_algorithm.imuHeading = 2.957;

// double distance = calculate_speads_algorithm.CoordinatesToMeters();
// double velocity = calculate_speads_algorithm.calculate_velocity();
// double angle = calculate_speads_algorithm.CoordinatesToAngle();
// Speeds Vs = calculate_speads_algorithm.calculate_speeds();
// cout << "distance: " << distance << endl;
// cout << "velocity: " << velocity << endl;
// cout << "angle: " << angle << endl;
// cout << "u: " << Vs.u << " ,v: " << Vs.v<< " ,r: " << Vs.r << endl;


// }
