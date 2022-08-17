#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
#include "ros/ros.h"

using namespace std;

struct Point
        {
        double x;
        double y;
        double z;
        };
struct Speeds
        {
        double u;
        double v;
        double r;
        };
class calculate_speads{
    public:

        double timeStamp0;
        double distance = 0;
        double velocity = 0;
        double timeStamp;
        double PI = 3.14159265;
        
       
        double Clatitude1 = 0;
        double Clongitude1 = 0;
        double Clatitude2 = 0;
        double Clongitude2 = 0;
        double imuHeading = 0;
        double imuHeading0 = 0;
        double gpsHeading = 0;
        // double latitudeR1;
        // double longitudeR1;
        // double latitudeR2;
        // double longitudeR2;

    double degreeToRadian(const double degree)
    {
        return (degree * PI / 180);
    };

    double radianToDegree(const double radian)
    {
        return (radian * 180 / PI);
    };



    double CoordinatesToMeters(double latitude1 , double longitude1 , double latitude2 , double longitude2)
    {
        
        latitude1 = degreeToRadian(latitude1);
        longitude1 = degreeToRadian(longitude1);
        latitude2 = degreeToRadian(latitude2);
        longitude2 = degreeToRadian(longitude2);

        double earthDiameterMeters = 12756000; 

        auto x = sin((latitude2 - latitude1) / 2), y = sin((longitude2 - longitude1) / 2);
        if (true) {
            distance = earthDiameterMeters * asin(sqrt((x * x) + (cos(latitude1) * cos(latitude2) * y * y)));
            return distance;
        
        }
        else{
            auto value = (x * x) + (cos(latitude1) * cos(latitude2) * y * y);
            distance = earthDiameterMeters * atan2(sqrt(value), sqrt(1 - value));
            return distance;
            }

    }



    double CoordinatesToAngle(double latitude1 , double longitude1 , double latitude2 , double longitude2)
    {
        const auto longitudeDifference = degreeToRadian(longitude2 - longitude1);
        latitude1 = degreeToRadian(latitude1);
        latitude2 = degreeToRadian(latitude2);



        const auto x = (cos(latitude1) * sin(latitude2)) -
                        (sin(latitude1) * cos(latitude2) * cos(longitudeDifference));
        const auto y = sin(longitudeDifference) * cos(latitude2);



        const auto degree = radianToDegree(atan2(y, x));
        //return (degree >= 0) ? degree : (degree + 360);
        gpsHeading = degreeToRadian(degree);
        return gpsHeading;
    }


    Point gpsToCoordinatesInMeter(double latitude1 , double longitude1 , double latitude2 , double longitude2)
    {
    Point p;



    auto angle = CoordinatesToAngle( latitude1 ,  longitude1 ,  latitude2 ,  longitude2);
    // cout << "Angle =  " << angle << endl;

    auto meters = CoordinatesToMeters( latitude1 ,  longitude1 ,  latitude2 ,  longitude2);
    // cout << "Meters = " << meters << endl;



    // auto coordinate = CoordinateToCoordinate(latitude1, longitude1, angle, meters);
    // cout << "Destination = (" << coordinate.first << "," << coordinate.second << ")\n";



    // p.x = meters * cos(degreeToRadian(angle));
    // p.y = meters * sin(degreeToRadian(angle));
    p.x = meters * cos(angle);
    p.y = meters * sin(angle);
    return p;
    }
    double calculate_velocity(){

        velocity = distance / (timeStamp - timeStamp0);
        return velocity;

    }
    Speeds calculate_speeds(){

        Speeds Vs;
        Vs.u = velocity*cos(imuHeading-gpsHeading);
        Vs.v = velocity*sin(imuHeading-gpsHeading);
        Vs.r = (imuHeading - imuHeading0)/(timeStamp - timeStamp0);
        return Vs;


    }
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
