#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
using namespace std;


#define NUM_INTERVALS (1000000)
#define NUM_BISECTION_ITERATIONS (30)

double ax = 0;
double ay = 0;
double xp = 3;
double yp = 3;
double xf = 10;
double yf = 10;
double delta = 1;
double m = (xf - xp)/(yf - yp);

#define MINX (0.8*ay)
#define MAXX (1.2*yf)

bool c1;
bool c2;
double xdF;
double ydF;
double f(double y){
    
    double x = ax + m*(y - ay);
    return pow(y - yp,2) + pow(x - xp,2) - pow(delta,2);
}

double bisection_method(double x0, double x1){
    for (unsigned int i = 0; i < NUM_BISECTION_ITERATIONS; i++){
        double midpoint = 0.5*x0 + 0.5*x1;
        f(x0) * f(midpoint) < 0 ? x1 = midpoint : x0 = midpoint;
    }
    return 0.5*x0 + 0.5*x1;
}

int main(int argc, char** argv){
    vector<pair<double, double>> relevant_intervals;
    for (unsigned int i = 0; i < NUM_INTERVALS - 1; i++){
        double x0 = MINX + (MAXX - MINX) / NUM_INTERVALS * (i);
        double x1 = MINX + (MAXX - MINX) / NUM_INTERVALS * (i + 1);
        if (f(x0) * f(x1) < 0)
            relevant_intervals.push_back(make_pair(x0, x1));
    }
    cout << fixed << setprecision(20);
    for (const auto & x : relevant_intervals){
        double yd = bisection_method(x.first, x.second);
        double xd = ax + m*(yd - ay);
        
        cout << "One solution is approximately, yd= " << yd <<" xd= "<< xd<< endl;
        if (xp <= xd <= xf | xp >= xd >= xf){
            c1 = true;
        }
        else{
            c1 = false;
        }

        if (yp <= yd <= yf | yp >= yd >= yf) {
            c2 = true;
        }
        else{
            c2 = false;
        }
        if (c1 & c2){
            int hehe = 1;
        }
        else {
            xd = xd + pow(xp - xd,2);
            yd = yd + pow(yp - yd,2);
        }
    if (xd > xf & xd > xp | xd < xf & xd < xp){
        xd = xf;
    }
    if (yd > yf & yd > yp | yd < yf & yd < yp){
        yd = yf;
    }
    xdF = xd;
    ydF = yd;

    }
    cout << "Final solution is yd= " << ydF << " and xd = " << xdF << endl;
}
