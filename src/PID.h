#ifndef PID_H
#define PID_H

#include <vector>

class PID {
  /*
  * Errors
  */

  double p_error;           // proportional error
  double i_error;           // integral error
  double d_error;           // derivative error
  double prev_cte;          // previous cross-track error
  double sum_squared_error; // sum of squared errors

  /*
  * Coefficients
  */
  std::vector<double> gains;  // [Kp, Ki, Kd]

  int iteration;

public:

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
  void Init(double Kp, double Ki, double Kd);

  const std::vector<double>& Gains();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Get iteration count.
  */
  int Iteration();

  /*
  * Calculate accumulated squared error.
  */
  double AccumulatedSquaredError();

  /*
  * Reset accumulated squared error and iteration count.
  */
  void Reset();

  /*
   *
   **/
  void UpdateGain(double delta, int gainIndex);
};

#endif /* PID_H */
