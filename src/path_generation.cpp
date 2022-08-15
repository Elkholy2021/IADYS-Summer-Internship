
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
}