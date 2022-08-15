
#include <iostream>
#include <cmath> 
#include <stdio.h>
#include<tuple> // for tuple
#define _USE_MATH_DEFINES
#include<string>

using namespace std;

#include <iomanip>
#include <vector>
#include <utility>

double u = 0.7;
double u_d = 0.5;
double psi =-0.23;
double psi_d = -0.231;
double e_psi = psi_d - psi;
double kp_u = 1;
double k_u_amax=5; //positive surge acceleration gain
double u_dot_max = 5; // maximum allowed surge acceleration
double u_d_yaw = 5; // desired surge velocity for periods of yaw motion

  double eta_u( double u,double u_d,double psi,double psi_d){
        //e_psi= psi_d-psi
        // e_psi=e_psi
    
    //   double u_d_ref = fmin( u_d_yaw+(u_d- u_d_yaw)*exp(-5.73-abs(e_psi)),u_d);
    //   cout << u_d_ref << endl;
    //   double u_d_dot = u_dot_max*tanh( k_u_amax*(u_d_ref-u)/ u_dot_max);
    //   double e_u = u - u_d;
    //   double eta_u = u_d_dot- kp_u*e_u;
      double eta_u=fmin(fmin(0.26,0.5),-0.3);
      return eta_u;
  }

  int main() {
  double e = eta_u(  u, u_d, psi, psi_d);
  cout << "eta_u: "<< e << endl;
  return 0;
}

