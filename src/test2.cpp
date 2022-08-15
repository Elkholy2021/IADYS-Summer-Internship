#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
using namespace std;











double f(double y,double ax,double ay,double xp,double yp,double xf,double yf,double delta){
    double m = (xf - xp)/(yf - yp);
    double x = ax + m*(y - ay);
    return pow(y - yp,2) + pow(x - xp,2) - pow(delta,2);
}

double bisection_method(double x0, double x1,double ax,double ay,double xp,double yp,double xf,double yf,double delta,int NUM_INTERVALS, int NUM_BISECTION_ITERATIONS){
    for (unsigned int i = 0; i < NUM_BISECTION_ITERATIONS; i++){
        double midpoint = 0.5*x0 + 0.5*x1;
        f(x0,ax,ay,xp,yp,xf,yf,delta) * f(midpoint,ax,ay,xp,yp,xf,yf,delta) < 0 ? x1 = midpoint : x0 = midpoint;
    }
    return 0.5*x0 + 0.5*x1;
}

void virtual_target(double ax,double ay,double xp,double yp,double xf,double yf,double delta, int NUM_INTERVALS, int NUM_BISECTION_ITERATIONS){
        #define MINX (0.8*ay)
        #define MAXX (1.2*yf)
        // #define NUM_INTERVALS (1000000)
        // #define NUM_BISECTION_ITERATIONS (30)
        #define NUM_INTERVALS (NUM_INTERVALS)
        #define NUM_BISECTION_ITERATIONS (NUM_BISECTION_ITERATIONS)
        bool c1;
        bool c2;
        double xdF;
        double ydF;
        double m = (xf - xp)/(yf - yp);

        vector<pair<double, double>> relevant_intervals;
    for (unsigned int i = 0; i < NUM_INTERVALS - 1; i++){
        double x0 = MINX + (MAXX - MINX) / NUM_INTERVALS * (i);
        double x1 = MINX + (MAXX - MINX) / NUM_INTERVALS * (i + 1);
        if (f(x0,ax,ay,xp,yp,xf,yf,delta) * f(x1,ax,ay,xp,yp,xf,yf,delta) < 0)
            relevant_intervals.push_back(make_pair(x0, x1));
    }
    //cout << fixed << setprecision(20);
    for (const auto & x : relevant_intervals){
        double yd = bisection_method(x.first, x.second, ax, ay, xp, yp, xf, yf, delta, NUM_INTERVALS,  NUM_BISECTION_ITERATIONS);
        double xd = ax + m*(yd - ay);
        
        //cout << "One solution is approximately, yd= " << yd <<" xd= "<< xd<< endl;
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
    //cout << "Final solution is yd= " << ydF << " and xd = " << xdF << endl;
}


int main(){

double ax = 0;
double ay = 0;
double xp = 5;
double yp = 4;
double xf = 10;
double yf = 8;
double delta = 2.5;
    virtual_target(ax,ay,xp,yp,xf,yf,delta,1000000,30);
}

