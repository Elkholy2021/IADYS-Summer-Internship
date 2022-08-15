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

void virtual_target2(double ax,double ay,double xp,double yp,double xf,double yf,double delta, int NUM_INTERVALS, int NUM_BISECTION_ITERATIONS){
        double a = yp;
        double b = xp -ax;
        double c = ay;
        double d = (xf - xp)/(yf - yp);
        double A = 1 + pow(d,2);
        double B = -2*a-2*b*d-2*c*pow(d,2);
        double C = pow(a,2) + pow(b,2) + 2*b*c*d + pow(c,2)*pow(d,2) - pow(delta,2);
        cout << "A: "<< A << endl;
        cout << "B: "<< B << endl;
        cout << "C: "<< C << endl;
        double xd;
        double yd;
        //eq2 = y^2 *A + y*B +C
        double slope;
        double ysol1;
        double ysol2;
        double xsol1;
        double xsol2;
        if (yf - yp == 0){
            yd = yp;

            if (xf >= xp){
                xd = xp + delta;
            }
            else{
                xd = xp - delta;
            }

            cout << "CASE 0" << endl;
            }
        else{
            cout << "CASE 00" << endl;

            slope = (xf - xp)/(yf - yp);
            ysol1 = (-B+sqrt(pow(B,2)-4*A*C))/(2*A);
            ysol2 = (-B-sqrt(pow(B,2)-4*A*C))/(2*A);
            xsol1 = ax + slope*(ysol1-ay);
            xsol2 = ax + slope*(ysol2-ay);
            cout << "sqrt: "<< sqrt(pow(B,2)-4*A*C) << endl;
            cout << "ysol1,ysol2: "<< ysol1 << ","<< ysol2<< endl;
            cout << "<ZZ ysol1,ysol2: "<< ysol1 << ","<< ysol2<< endl;
        cout << "xsol1,ysol1: "<< xsol1 << ","<< ysol1<< endl;
        cout << "xsol2,ysol2: "<< xsol2 << ","<< ysol2<< endl;
        bool c1;
        bool c2;
        double xdF;
        double ydF;
        for (int i = 0; i < 2; i++) {
            if (i ==0){
            yd = ysol1;
            xd = xsol1;
            }
            else if (i ==1){
            yd = ysol2;
            xd = xsol2;
            }

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
            if (c1 == true & c2 == true){
                int hehe = 1;
                cout << "CASE 1" << endl;
                break;
                cout << "FFF "<< i<< " xd,yd: " << xd <<","<<yd<<endl; 

            }
            else {
                xd = xd + pow(xp - xd,2);
                yd = yd + pow(yp - yd,2);
                cout << "CASE 2" << endl;
            }
            if (xd > xf & xd > xp ){
                xd = xf;
                cout << "CASE 3" << endl;
            }
            else if (xd < xf & xd < xp) {
                xd = xp;
                cout << "CASE 32" << endl;

            }
            if (yd > yf & yd > yp){
                yd = yf;
                cout << "CASE 4" << endl;
            }
            else if (yd < yf & yd < yp){
                yd = yp;
            cout << "CASE 42" << endl;
            }

            // xdF = xd;
            // ydF = yd;

        }

            }
        

    //   xd = xdF;
    //   yd = ydF;
    cout << "xd,yd: " << xd <<","<<yd<<endl; 

    ////cout << "Final solution is yd= " << ydF << " and xd = " << xdF << endl;
  }
int main(){
double ax = 3;
double ay = 3;
double xp = 3.35535;
double yp = 3;
double xf = 10;
double yf = 3;
double delta = 1;
virtual_target2(ax,ay,xp,yp,xf,yf,delta,1000000,30);
}

