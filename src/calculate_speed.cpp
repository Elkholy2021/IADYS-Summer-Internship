Hello
Hello again
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
using namespace std;



class calculate_speads{
    public:
        double x0;
        double y0;
        double timeStamp0;
        double x,y;
        double distance;
        double velocity;
    void calculate_spead(double heading, double lat,double lon,double timeStamp){
        //x,y = GPStoMETER(lat,lon);
        distance = sqrt(pow(y-y0,2)+pow(x-x0,2));
        velocity = distance / (timeStamp - timeStamp0);






    }
}
