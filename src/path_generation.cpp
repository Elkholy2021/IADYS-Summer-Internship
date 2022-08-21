
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
#define points_per_side 6
struct demo{
        //array declared inside structure
        double arr[2][points_per_side+2];
      };
double W = 20;
double L = 8;
double R = 3;

class path_generation{        
  public:
      double W , L , R , x0 , y0 , alpha;
      //int points_per_side = 6; 
      double x , y;
      bool other_side;
      
      
      
 
  struct demo generate_points(){
      
      struct demo points;
      points.arr[0][0] = 0;
      points.arr[1][0] = 0;
      x = x0;
      y = y0;
      other_side = true;
      for (int i = 0; i <= points_per_side; ++i) {
        //cout << i << " ";
        points.arr[0][i+1] = x;
        points.arr[1][i+1] = y;
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
  return points;
  }
};
int main(){

  path_generation path_generation;
  struct demo points;
  path_generation.W = W;
  path_generation.L = L;
  path_generation.R = R;
  path_generation.x0 = 2;
  path_generation.y0 = 2;
  path_generation.alpha = 0;
  //path_generation.init_path();
  points = path_generation.generate_points();
  for(int i=0;i<points_per_side+2;i++)
	{
		cout<<points.arr[0][i]<<","<<points.arr[1][i]<<endl;;
	}
  return 0;
  
}