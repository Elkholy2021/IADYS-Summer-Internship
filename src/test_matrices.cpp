
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



void thruster_forcesV2(double tau_x,double tau_y,double tau_N,bool clipping){
      //B = 0.538
      //tau_L=0.5*tau_x
      //tau_R=0.5*tau_y
      int r1 = 3;
      int c1 = 3;
      int r2 = 3;
      int c2 = 1;
      double C = 0;
      double B = 0.538;
      double f[r1][c1] = {{1,1,0},
                        {0,0,1},  
                        {0.5*B,-0.5*B,C}};
      
      double h[r2][c2] = {{tau_x},{tau_y},{tau_N}};
      //cout << "dd"<< h[2][0] <<  endl;
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
      tuple<double,double,double> thrusts;

      thrusts = make_tuple(taus[0][0],taus[1][0],taus[2][0]);
      cout << "calculated taus: " << taus[0][0]<< " "<< taus[1][0] << " " << taus[2][0] << endl;
      
  }

  int main(){

    thruster_forcesV2(-42.7122 ,-0.428598, 13.1398,true);

    return 0;
}