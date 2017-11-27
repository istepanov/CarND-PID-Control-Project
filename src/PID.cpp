#include "PID.h"
#include <math.h>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    gains = {Kp, Ki, Kd};

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
    sum_squared_error = 0.0;

    iteration = 0;
}

const std::vector<double>& PID::Gains() {
  return gains;
}

void PID::UpdateError(double cte) {
    p_error = cte;              // proportional error
    i_error += cte;             // integral error
    d_error = cte - prev_cte;   // derivative error
    prev_cte = cte;
    sum_squared_error += pow(cte, 2.0);
    iteration++;
}

double PID::TotalError() {
    return gains[0] * p_error + gains[1] * i_error + gains[2] * d_error;
}

int PID::Iteration() {
  return iteration;
}

double PID::AccumulatedSquaredError() {
  if (iteration == 0) {
    return 0;
  } else {
    return sum_squared_error / (double)iteration;
  }
}

void PID::Reset() {
  sum_squared_error = 0.0;
  iteration = 0;
}

void PID::UpdateGain(double delta, int gain_index) {
  gains[gain_index] += delta;
}
