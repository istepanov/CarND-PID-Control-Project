#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <vector>

class Twiddle {
  PID& pid_;
  std::vector<double> gain_deltas_;
  std::vector<double> initial_gain_deltas_;

  int current_gain_index_;
  double best_error_;
  int current_case_;

  void MoveToNextGainIndex();

public:

  /*
  * Constructor
  */
  Twiddle(PID& pid, double d_Kp, double d_Ki, double d_Kd);

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Update PID gains between simulation runs.
  */
  bool Iterate();

  /*
  * Destructor.
  */
  void Reset();

  /*
  * Get PID gain deltas (for debugging)
  */
  std::vector<double> GainDeltas();
};

#endif /* TWIDDLE_H */
