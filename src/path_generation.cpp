
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

class path_generation {        
  public:
      double W , L , R , x0 , y0 , alpha;
      int points_per_side; 
      double x , y;
      bool other_side;
      
  void init_path(){
      points_per_side = floor(W/R);
      
  }
  void generate_points(){
      
      double points[2][floor(W/R)*2+1];

      points[0][0] = 0;
      points[1][0] = 0;
      x = x0;
      y = y0;
      other_side = true;
      for (int i = 0; i <= points_per_side; ++i) {
        //cout << i << " ";
        points[0][i+1] = x;
        points[1][i+1] = y;
        if (other_side == true){
          if (x == x0 + L){
            x = x - L;
            y = y;
            other_side = false;
          }
          else{
            x = x + L;
            y = y;
            other_side = false;
          }
        }
        else{
          x = x;
          y = y + R;
          other_side = true;
        }



    }

  }
};
int main(){
  path_generation path_generation;
  path_generation.W = 20;
  path_generation.L = 8;
  path_generation.R = 3;
  path_generation.x0 = 2;
  path_generation.y0 = 2;
  path_generation.alpha = 0;
  path_generation.init_path();
  path_generation.generate_points();
  for (int i = 0; i <= floor(path_generation.W/path_generation.R); ++i) {
    cout << path_generation.points[0][i+1] << "," << path_generation.points[1][i+1]<< endl;;
}