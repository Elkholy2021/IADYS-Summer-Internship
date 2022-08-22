#include <iostream>
#include <cmath> 
#include <stdio.h>
#include<tuple> // for tuple
#define _USE_MATH_DEFINES
#include<string>

using namespace std;
#include "ros/ros.h"

#include <iomanip>
#include <vector>
#include <utility>


class control_jellyfishbot {        
  public:
      double m= 18; //kg
      double Iz= 2.37; //inertia around z
      double dt=0.1; //time interval in seconds, depending on the frequency of the sensor readings
      
      double t_final=10;
      double B = 0.538; //distance between the left and the right thrusters
      double C = 0; //ditance between middle thruster and center of gravity

      double Xu = 2.245; //2.245 // linear drag coeff along x-direction - surge motion
      double Xuu = 22; //19.05 // non-linear drag coeff along x-direction - surge motion
      double Yv = 0.19; //-0.1972 // linear drag coeff along y-direction - sway motion
      double Yvv = 78; //71.74 // non-linear drag coeff along y-direction - sway motion
      double Nr = 2.591; //-0.01391 // linear drag coeff around z-direction - yaw motion
      double Nrr = 3; //2.591 // non-linear drag coeff around z-direction - yaw motion
      //Dx= Xu*u+Xuu*|u|*u

      double kp_u = 1; //25 //proportional gain in the surge contoller
      double kp_v = 0.5; //proportional gain in the sway contoller
      double kp_psi = 2.5; //2.5; //proportional gain in the yaw heading contoller
      double kd_psi = 2.5; //derivative gain in the yaw heading contoller
      double ki_psi = 2; //integral gain in the yaw heading contoller
      double e_psi_integral = 0;
      ros::Publisher pub_thrust_l;
      ros::Publisher pub_thrust_r;
      ros::Publisher pub_thrust_tt;
      ros::Publisher pp_topic;
      ros::Publisher vt_topic;
      ros::Publisher velocities_topic;

      double k_u_amax=5; //positive surge acceleration gain
      double u_dot_max = 5; // maximum allowed surge acceleration
      double u_d_yaw = 5; // desired surge velocity for periods of yaw motion

      double k_v_amax=0.1; //positive sway acceleration gain
      double v_dot_max = 0.1; // maximum allowed sway acceleration
      double v_d_yaw = 0.1; // desired sway velocity for periods of yaw motion

      double e_psi0 = 0;
      double e_psi = 0;

      double e_integral = 0;
      double K_P = 1;
      double K_I = 0.25;
      double Beta = 0;////the sideslip angle (drift) which can be neglected to simplify steering law

  
      double y_k1 = 0;
      double y_k = 0;
      double x_k1 = 0;
      double x_k = 0;
      double xf = 10000;
      double yf = 10000;
      double xf0 = 0;
      double yf0 = 0;
      double distance = 1000;
      bool enable_print = true;

      double xd = 9999;
      double yd = 9999;
      double u_d = 0;
      double v_d = 0;


      double Yp = 0;
      double ax = 0;
      double ay = 0;
      double delta =0;

      double xp = 0;
      double yp = 0;
      double slope = 0;
      bool MATH_ERROR = false;
      bool getting_away = false;
      double distance0 = 10000;
      int sign = 1;
      int clip_counter = 0;
      int counter = 0;
      string movement;
      double x = 0;
      double y = 0;
      //double e_dot_psi;
      //double Eta_psi;
      bool stop = false ;//A flag for check if the robot arrived to the final point and should stop
    
      tuple <double, double, double> thrusts;
      float tau_L,tau_R,tau_M;
      double psi;
      //double psi_d;
      double u;
      double v;
      double r;
  double tau_x(double eta_u,double u) {
      double X_drag = Xu*u + Xuu*abs(u)*u;
      double tau_X = m*eta_u + X_drag;
      return tau_X;
  }

  double tau_y(double eta_v,double v){
      double Y_drag = Yv*v + Yvv*abs(v)*v;
      double tau_Y = m*eta_v + Y_drag;
      return tau_Y;
  }

  double tau_N(double eta_psi,double r){
      double N_drag = Nr*r + Nrr*abs(r)*r;
      double tau_n = Iz*eta_psi + N_drag;
        //cout << "Iz: "<< Iz << endl;

      cout << "eta_psi: "<< eta_psi << endl;
      return tau_n;  
  }



  double eta_u( double u,double u_d,double psi,double psi_d){
        //e_psi= psi_d-psi
        // e_psi=e_psi
        // cout << "MM e_psi: "<< e_psi << endl;
        // cout << "MM u_d_yaw: "<< u_d_yaw << endl;
        // cout << "MM u_d: "<< u_d << endl;
        // cout << "MM u: "<< u << endl;
        
      double u_d_ref = fmin( u_d_yaw+(u_d- u_d_yaw)*exp(-5.73-abs(e_psi)),u_d);
      double u_d_dot = u_dot_max*tanh( k_u_amax*(u_d_ref-u)/ u_dot_max);
      double e_u = u - u_d;
      double eta_U = u_d_dot- kp_u*e_u;
    //   if (eta_u < 0){
    //       eta_u = 0;
    //   }
      return eta_U;
  }

  double eta_v(double v,double v_d,double psi,double psi_d){
      //e_psi= psi_d-psi
      double v_d_ref = fmin( v_d_yaw+(v_d- v_d_yaw)*exp(-5.73-abs( e_psi)),v_d);
      double v_d_dot = v_dot_max*tanh( k_v_amax*(v_d_ref-v)/ v_dot_max);
      double e_v = v_d- v;
      double eta_V = v_d_dot- kp_v*e_v;
      return eta_V;
  }


  double eta_psi(double v,double v_d,double psi,double psi_d){
      //e_psi= psi_d-psi
        cout << "e_psi: "<< e_psi <<endl;
        //cout << "e_psi0: "<< e_psi0 <<endl;
        
        double e_dot_psi = ( e_psi- e_psi0)/dt;
        double E_psi = e_psi-e_psi0;
        e_psi_integral = e_psi_integral + E_psi*dt;
        double Eta_psi = -kp_psi*e_psi - kd_psi*e_dot_psi - ki_psi*e_psi_integral;
        e_psi0 = e_psi;
        //cout << "e_psi: "<< e_psi <<endl;
        //cout << "e_psi0: "<< e_psi0 <<endl;
        //cout << "e_dot_psi: "<< e_dot_psi <<endl;
        //cout << "e_psi: "<< E_psi <<endl;
      return Eta_psi;
  }
  // struct SS {
  //   double tau_L, tau_R, tau_M;};
  // typedef struct SS Struct;

 
  void thruster_forcesV2(double tau_x,double tau_y,double tau_N,bool clipping){
      //B = 0.538
      //tau_L=0.5*tau_x
      //tau_R=0.5*tau_y
      int r1 = 3;
      int c1 = 3;
      int r2 = 3;
      int c2 = 1;
      
      double f[3][3] = {{1,1,0},
                        {0,0,1},  
                        {0.5*B,-0.5*B,C}};
      
      double h[3][1] = {{tau_x},{tau_y},{tau_N}};
      ////cout << "dd"<< h[2][0] <<  endl;
      double f_inverse[3][3];
      float determinant = 0;
      int i;
      int j;
      int k;
      for(i = 0; i < 3; i++){
      determinant = determinant + (f[0][i] * (f[1][(i+1)%3] * f[2][(i+2)%3] - f[1][(i+2)%3] * f[2][(i+1)%3]));
      }

      for(i = 0; i < 3; i++){
      for(j = 0; j < 3; j++){
          f_inverse[i][j] = ((f[(j+1)%3][(i+1)%3] * f[(j+2)%3][(i+2)%3]) - (f[(j+1)%3][(i+2)%3] * f[(j+2)%3][(i+1)%3]))/ determinant;
          
          }
      }

      double taus [3][1];
      for (i = 0; i < r1; i++) {
          for (j = 0; j < c2; j++) {
              taus[i][j] = 0;
              for (k = 0; k < c1; k++)
                  taus[i][j] += f_inverse[i][k] * h[k][j];
          }
      }

      //thrusts = make_tuple(taus[0][0],taus[1][0],taus[2][0]);
      tau_L = taus[0][0];
      tau_R = taus[1][0];
      tau_M = taus[2][0];
      if (clipping == true){
        double tau_max = fmax(fmax(tau_L,tau_R),tau_M);
        double tau_min = fmin(fmin(tau_L,tau_R),tau_M);
        if (tau_max > abs(tau_min) & tau_max > 30){
            tau_L =  (tau_L/tau_max)*30;
            tau_R =  (tau_R/tau_max)*30;
            tau_M =  (tau_M/tau_max)*30;
        }
        if (tau_max < abs(tau_min) & tau_min < -30){
            tau_L =  (tau_L/abs(tau_min))*30;
            tau_R =  (tau_R/abs(tau_min))*30;
            tau_M =  (tau_M/abs(tau_min))*30;
            }

      }
      
  }


  void projected_point(){
      double a = x - ax;
      double b = y - ay;
      double c = xf - ax;
      double d = yf - ay;
      double dot = a*c + b*d;
      double len_sq = c * c + d * d;
      double xx;
      double yy;
      double param = -1;
      if (len_sq != 0){ // #// in case of 0 length line
          param = dot / len_sq;
          ////cout << "I am here: " << param << endl;
        if (param < 0){
            xx = ax;
            yy = ay;
        }
        else if (param > 1){
            xx = xf;
            yy = yf;
        }
        else{
            xx = ax + param * c;
            yy = ay + param *d;
        }
      }
        //print("xx,yy = {},{}".format(xx,yy))
        xp = xx;
        yp = yy;
        // //cout << "FF param: "<< param << endl; 
        // //cout << "FF dot,len: "<< dot << ","<<len_sq<< endl;
        // //cout << "FF xx,yy: "<< xx << ","<<yy<< endl;
        // //cout << "FF xp,yp: "<< xp << ","<<yp<< endl;
        // //cout << "FF x,y: "<< x << ","<<y<< endl;
        // //cout << "FF a,b: "<< a << ","<<b<< endl;
  }

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
          #define MINX (0.9*ay)
          #define MAXX (1.1*yf)
          // #define NUM_INTERVALS (1000000)
          // #define NUM_BISECTION_ITERATIONS (30)
          #define NUM_INTERVALS (NUM_INTERVALS)
          #define NUM_BISECTION_ITERATIONS (NUM_BISECTION_ITERATIONS)
          bool c1;
          bool c2;
          double xdF;
          double ydF;
          double slope = (xf - xp)/(yf - yp);
        cout << "xf,yf: "<< xf<<","<<yf<< endl;
          vector<pair<double, double>> relevant_intervals;
      for (unsigned int i = 0; i < NUM_INTERVALS - 1; i++){
          double x0 = MINX + (MAXX - MINX) / NUM_INTERVALS * (i);
          double x1 = MINX + (MAXX - MINX) / NUM_INTERVALS * (i + 1);
          if (f(x0,ax,ay,xp,yp,xf,yf,delta) * f(x1,ax,ay,xp,yp,xf,yf,delta) < 0)
              relevant_intervals.push_back(make_pair(x0, x1));
      }
      ////cout << fixed << setprecision(20);
      for (const auto & x : relevant_intervals){
          double yd = bisection_method(x.first, x.second, ax, ay, xp, yp, xf, yf, delta, NUM_INTERVALS,  NUM_BISECTION_ITERATIONS);
          double xd = ax + slope*(yd - ay);
          
          ////cout << "One solution is approximately, yd= " << yd <<" xd= "<< xd<< endl;
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
      if (((xd > xf) && (xd > xp)) || ((xd < xf) && (xd < xp))){
          xd = xf;
      }
      if (((yd > yf) && (yd > yp)) || ((yd < yf) && (yd < yp))){
          yd = yf;
      }
      xdF = xd;
      ydF = yd;

      }
      xd = xdF;
      yd = ydF;
    ////cout << "Final solution is yd= " << ydF << " and xd = " << xdF << endl;
  }
  void virtual_target2(double ax,double ay,double xp,double yp,double xf,double yf,double delta, int NUM_INTERVALS, int NUM_BISECTION_ITERATIONS){
        double a = yp;
        double b = xp -ax;
        double c = ay;
        double d = (xf - xp)/(yf - yp);
        double A = 1 + pow(d,2);
        double B = -2*a-2*b*d-2*c*pow(d,2);
        double C = pow(a,2) + pow(b,2) + 2*b*c*d + pow(c,2)*pow(d,2) - pow(delta,2);
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
            }
        else if (xf - xp == 0){
            xd = xp;
            if (yf >= yp){
                yd = yp + delta;
            }
            else {
                yd = yp - delta;
            }

        }
        else{
            slope = (xf - xp)/(yf - yp);
            ysol1 = (-B+sqrt(pow(B,2)-4*A*C))/(2*A);
            ysol2 = (-B-sqrt(pow(B,2)-4*A*C))/(2*A);
            xsol1 = ax + slope*(ysol1-ay);
            xsol2 = ax + slope*(ysol2-ay);
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
                break;
            }
            else {
                xd = xd + pow(xp - xd,2);
                yd = yd + pow(yp - yd,2);
            }
            

            // xdF = xd;
            // ydF = yd;

        }

            }
        if (xd >= xf && xd >= xp ){
                if (xf >= xp){
                    xd = xf;
                }
                else{
                    xd = xp;
                }

            }
            else if (xd < xf && xd < xp){
                if (xf < xp){
                    xd = xf;
                }
                else{
                    xd = xp;
                }
            }
            if (yd > yf && yd > yp ){
                if (yf >= yp){
                    yd = yf;
                }
                else{
                    yd = yp;
                }
            }
            else if (yd < yf && yd < yp){
                if (yf < yp){
                    yd = yf;
                }
                else{
                    yd = yp;
                }
            }
        
    //   xd = xdF;
    //   yd = ydF;
    cout << "xd,yd: " << xd <<","<<yd<<endl; 

    ////cout << "Final solution is yd= " << ydF << " and xd = " << xdF << endl;
  }
  bool check_arrival(double threshold){
        distance = sqrt(pow(xf - x,2) + pow(yf-y,2));
        if (distance <= threshold){
            return true;
        }
        else{
            return false;
        }
  }

  double force_to_pwm(double force){
      double pwm, p1,p2,p3;
      if (force <= 0){
            p1 =       15.76;  
            p2 =       163.3;  
            p3 =        1453; 
            pwm = p1*pow(force,2) + p2*force + p3;
      }
      else{
            p1 =      -10.45;  
            p2 =       130.4;  
            p3 =        1548;  
            pwm = p1*pow(force,2) + p2*force + p3;
      }
      return pwm;
  }      

  void obtain__thruster_commands_LOS_Virtual_target(double u_d, double v_d, double xd, double yd){
        
        double Xp = atan2(yd-y,xd-x);
        //Xp = math.atan2(self.yd-self.yp,self.xd-self.xp)
        cout <<"x,y: "<< x<<","<<y<<" xd,yd: "<< xd <<","<<yd<<endl; 
        cout <<"xp,yp: "<< xp<<","<<yp <<endl; 
        double e = -(x-xd)*sin(Xp)+(y-yd)*cos(Xp);
        //e = -(self.x-self.xp)*math.sin(Xp)+(self.y-self.yp)*math.cos(Xp)


        e_integral =  e_integral + e*dt;
        double Xr = atan(-K_P*e-K_I*e_integral);
       
        double psi_d = Xp + Xr; // - atan(self.v/u_d)
        cout<< "Xp: "<< Xp<< " ,Xr: "<< Xr<< endl;
        cout<< "psi_d before: "<< psi_d<< endl;
        cout << "psi before: "<<psi << endl;
        if (psi_d < -0.5*M_PI  | psi < -0.5*M_PI  | psi_d > 0.5*M_PI  | psi > 0.5*M_PI ){
            if (psi_d < 0){
              psi_d = psi_d +2*M_PI;
            }
                
            
            if (psi < 0){
              psi = psi + 2*M_PI; 
            }
        }

        //cout << "psi_d: "<< psi_d<< endl; 
        //cout << "FFF psi_d: " << psi_d << endl; 
        double e_psi_candidate = psi_d - psi;
        cout << "psi: "<< psi<< " ,psi_d: "<< psi_d << endl;
        cout << "e_psi_candidate before: "<< e_psi_candidate << endl;
        if (e_psi_candidate > M_PI){
            e_psi_candidate = e_psi_candidate -M_PI;
        }
        else if (e_psi_candidate < -M_PI){
            e_psi_candidate = e_psi_candidate + M_PI;
        }
        e_psi = e_psi_candidate;
        //cout << "FFF e_psi: " << e_psi << " yaw: "<< psi << endl; 


        
        if (e_psi >= 0){
            //MOVE RIGHT
            movement = "RIGHT";
        }
        else if (e_psi < 0){
            //MOVE LEFT
            movement = "LEFT";
        }
        double eta_uV= eta_u(u,u_d,psi,psi_d);
        double tau_xV = tau_x(eta_uV,u);

        
        double eta_vV = eta_v(v,v_d,psi,psi_d);
        double tau_yV = tau_y(eta_vV,v);

        double eta_psiV = eta_psi(v,v_d,psi,psi_d);
        double tau_NV = tau_N(eta_psiV,r);
        //tau_xV = 7;
        //tau_yV = 0;
        cout << "etas calculated: "<<eta_uV<< " "<< eta_vV<< " "<<eta_psiV<< endl;
        cout << "taus required: "<<tau_xV<< " "<< tau_yV<< " "<<tau_NV<< endl;
        thruster_forcesV2(tau_xV,tau_yV,tau_NV,true);
        
        cout << "taus calculated: "<<tau_R<< " "<< tau_L<< " "<<tau_M<< endl;
        if (tau_R <= 1.1*tau_L &  tau_R >= 0.9*tau_L){
            // MOVE FORWARD
            movement = "FORWARD";
        }
        cout << "MOVING  " << movement <<endl;
        if (stop != true){
            //tau_L,tau_R,tau_M = round(tau_L,1) ,round(tau_R,1) ,round(tau_M,1);
            double haha = 555;
            //cout << "S1" << endl;
        }
        else{
            tau_L = 0;
            tau_R = 0;
            tau_M = 0;
            //cout << "S2" << endl;
        }
 
        //return tau_L,tau_R,tau_M


  }



};


// int main() {
//   // struct SS {
//   //   double tau_L, tau_R, tau_M;};
//   //typedef struct SS Struct;
//   tuple < double, double , double > thrusts;
//   control_jellyfishbot jellyfishbot_control_system;
//   //cout << "kp_psi " << jellyfishbot_control_system.kp_psi << endl;
//   //cout << "tau_x " << jellyfishbot_control_system.tau_x(20.2,5) << endl;
//   //cout << "enable_print " << jellyfishbot_control_system.enable_print << endl;
//   jellyfishbot_control_system.thruster_forcesV2(8,0,6.5,true);
//   thrusts =  jellyfishbot_control_system.thrusts;
//   //cout << get<0>(thrusts) << " "<< get<1>(thrusts) << " "<<  get<2>(thrusts) << endl;
//   return 0;
// }

