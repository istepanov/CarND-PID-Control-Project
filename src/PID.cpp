#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(
    double Kp, double Ki, double Kd, double d_Kp, double d_Ki, double d_Kd
) {
    gains = {Kp, Ki, Kd};

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
}

void PID::UpdateError(double cte) {
    p_error = cte;              // proportional error
    i_error += cte;             // integral error
    d_error = cte - prev_cte;   // derivative error
    prev_cte = cte;
}

double PID::TotalError() {
    return gains[0] * p_error + gains[1] * i_error + gains[2] * d_error;
}

void PID::Twiddle() {

}
