#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>

using namespace std;


class convert_gps_to_meter{
    public:

        double distance = 0;
        double velocity = 0;
        double PI = 3.14159265;
        
       
        double Clatitude1 = 0;
        double Clongitude1 = 0;
        double Clatitude2 = 0;
        double Clongitude2 = 0;
        double imuHeading;
        double imuHeading0 ;
        double gpsHeading = 0;
        double heading_correction;
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

        velocity = distance /1; //gps freq is 1hz
        return velocity;

    }
    
};

// convert_gps_to_meter conversion;
// int main(){



// demo gpsPoints;
// demo meterPoints;
// meterPoints.arr[0][0] = 0;
// meterPoints.arr[1][0] = 0;

// gpsPoints.arr[0][0] = 43.21355;
// gpsPoints.arr[1][0] =  5.536307;

// gpsPoints.arr[0][1] = 43.213892;
// gpsPoints.arr[1][1] = 5.536114;

// gpsPoints.arr[0][2] = 43.213923;
// gpsPoints.arr[1][2] = 5.536211;

// gpsPoints.arr[0][3] = 43.213612;
// gpsPoints.arr[1][3] = 5.536380;

// gpsPoints.arr[0][4] = 43.213900;
// gpsPoints.arr[1][4] = 5.536200;

// conversion.Clatitude1 = gpsPoints.arr[0][0];
// conversion.Clongitude1 = gpsPoints.arr[1][0];
// for (int i = 1; i < 5; ++i) {
//     //cout << "i: "<< i<< endl;
//     conversion.Clatitude2 = gpsPoints.arr[0][i]; 
//     conversion.Clongitude2 = gpsPoints.arr[1][i];  
//     Point pm = conversion.gpsToCoordinatesInMeter(conversion.Clatitude1,conversion.Clongitude1,conversion.Clatitude2,conversion.Clongitude2);
//     meterPoints.arr[0][i] = pm.x;
//     meterPoints.arr[1][i] = pm.y;
// }
// for (int i = 0; i < 5; i++){
//     cout <<"x,y: "<<meterPoints.arr[0][i] <<","<<meterPoints.arr[1][i]<<endl;
// }




// }
