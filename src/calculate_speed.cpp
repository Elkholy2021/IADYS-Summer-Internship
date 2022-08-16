#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
using namespace std;



class calculate_speads{
    public:

        double timeStamp0 = 10;
        double distance = 0;
        double velocity = 0;
        double timeStamp = 0;
        double PI = 3.14159265;
        struct Point
        {
        double x;
        double y;
        double z;
        };
       
        double latitude1 = 0;
        double longitude1 = 0;
        double latitude2 = 0;
        double longitude2 = 0;

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



    double CoordinatesToAngle(double latitude1,
                            const double longitude1,
                            double latitude2,
                            const double longitude2)
    {
    const auto longitudeDifference = degreeToRadian(longitude2 - longitude1);
    latitude1 = degreeToRadian(latitude1);
    latitude2 = degreeToRadian(latitude2);



    const auto x = (cos(latitude1) * sin(latitude2)) -
                    (sin(latitude1) * cos(latitude2) * cos(longitudeDifference));
    const auto y = sin(longitudeDifference) * cos(latitude2);



    const auto degree = radianToDegree(atan2(y, x));
    return (degree >= 0) ? degree : (degree + 360);
    }


    Point gpsToCoordinatesInMeter(double latitude1, double longitude1, double latitude2, double longitude2)
    {
    Point p;



    auto angle = CoordinatesToAngle(latitude1, longitude1, latitude2, longitude2);
    // cout << "Angle =  " << angle << endl;

    auto meters = CoordinatesToMeters();
    // cout << "Meters = " << meters << endl;



    // auto coordinate = CoordinateToCoordinate(latitude1, longitude1, angle, meters);
    // cout << "Destination = (" << coordinate.first << "," << coordinate.second << ")\n";



    p.x = meters * cos(degreeToRadian(angle));
    p.y = meters * sin(degreeToRadian(angle));
    return p;
    }
    double calculate_spead(){

        velocity = distance / (timeStamp - timeStamp0);
        return velocity;

    }
};

calculate_speads calculate_speads_algorithm;

int main(){
struct PointGPS
    {
    double lat;
    double lon;
    };
PointGPS P0;
PointGPS P;


calculate_speads_algorithm.latitude1 = 43.213552;
calculate_speads_algorithm.longitude1 = 5.536321;
calculate_speads_algorithm. latitude2 = 43.213897;
calculate_speads_algorithm.longitude2 = 5.536085;
calculate_speads_algorithm.timeStamp = 18.5782;
double distance = calculate_speads_algorithm.CoordinatesToMeters();
double velocity = calculate_speads_algorithm.calculate_spead();

cout << "distance: " <<distance << endl;
cout << "velocity: " <<velocity << endl;

}
