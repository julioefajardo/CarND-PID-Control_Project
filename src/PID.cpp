#include "PID.h"
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std;

/*
* TODO: Complete the PID class.
  Generic PID implementation taking into account time derivative (dt).
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Initialize PID hyperparameters and PID errors. 
  // NOTE: Consult PID.h for more information about this method (and others in this file).	
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  // Time initialization verification
  is_t_Init = false;

}

void PID::UpdateError(double cte) {
  // Update PID errors given cte and get time measurements (t,dt). 
  if(!is_t_Init)
    t = std::chrono::system_clock::now(); 
  else{
    dt = std::chrono::system_clock::now() - t;
    t  = std::chrono::system_clock::now();
    d_error = (cte - p_error)/dt.count();
    p_error = cte;
    i_error += cte*dt.count();
  }
}

double PID::TotalError() {
  // Get PID correction, returns 0 until time measurements are available.
  if (!is_t_Init){
    is_t_Init = true;
    return 0;
  }
  else  
    return -Kp*p_error - Kd*d_error - Ki*i_error;
}

