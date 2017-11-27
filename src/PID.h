#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */

  double p_error;   // proportional error
  double i_error;   // integral error
  double d_error;   // derivative error
  double prev_cte;  // previous cross-track error

  /*
  * Coefficients
  */
  std::vector<double> gains;  // [Kp, Ki, Kd]

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(
    double Kp, double Ki, double Kd,
    double d_Kp = 0.0, double d_Ki = 0.0, double d_Kd = 0.0
  );

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   *
   **/
  void Twiddle();
};

#endif /* PID_H */
