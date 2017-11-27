#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include "PID.h"

class Twiddle {
  PID& pid_;
  std::vector<double> gain_deltas_;

  int current_gain_index_;
  double best_error_;
  int current_case_;

  void MoveToNextGainIndex();

public:

  /*
  * Constructor
  */
  Twiddle(PID &p);

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  double BestError();

  double GainDeltaSum();

  bool Iterate();

  void Reset();
};

#endif /* TWIDDLE_H */
