#include "PID.h"
#include <iostream>
#include <algorithm>

using std::min;
using std::max;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error = i_error + cte;
}

double PID::TotalError() {
  double totalError = -(Kp * p_error + Kd * d_error + Ki * i_error);
  return std::min(1.0, std::max(totalError, -1.0));
}
