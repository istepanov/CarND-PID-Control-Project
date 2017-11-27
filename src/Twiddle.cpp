#include "Twiddle.h"
#include <math.h>
#include <numeric>

Twiddle::Twiddle(PID& pid, double d_Kp, double d_Ki, double d_Kd): pid_(pid) {
    initial_gain_deltas_ = std::vector<double> {d_Kp, d_Ki, d_Kd};
    gain_deltas_.resize(3);
    this->Reset();
}

Twiddle::~Twiddle() {}

void Twiddle::Reset() {
  for (int i = 0; i < initial_gain_deltas_.size(); i++) {
    gain_deltas_[i] = initial_gain_deltas_[i];
  }
  current_gain_index_ = 0;
  best_error_ = -1.0;
  current_case_ = 0;
}

void Twiddle::MoveToNextGainIndex() {
  current_gain_index_++;
  if (current_gain_index_ >= gain_deltas_.size()) {
    current_gain_index_ = 0;
  }
}

bool Twiddle::Iterate() {
  if (best_error_ < 0.0) {
    best_error_ = pid_.AccumulatedSquaredError();
  }
  switch (current_case_) {
    case 0: {
      pid_.UpdateGain(gain_deltas_[current_gain_index_], current_gain_index_);
      current_case_ = 1;
      break;
    }
    case 1: {
      double error = pid_.AccumulatedSquaredError();
      if (error < best_error_) {
        best_error_ = error;
        gain_deltas_[current_gain_index_] *= 1.1;
        current_case_ = 0;
        this->MoveToNextGainIndex();
      } else {
        pid_.UpdateGain(-2.0 * gain_deltas_[current_gain_index_], current_gain_index_);
        current_case_ = 2;
      }
      break;
    }
    case 2: {
      double error = pid_.AccumulatedSquaredError();

      if (error < best_error_) {
        best_error_ = error;
        gain_deltas_[current_gain_index_] *= 1.1;
      } else {
        pid_.UpdateGain(gain_deltas_[current_gain_index_], current_gain_index_);
        gain_deltas_[current_gain_index_] *= 0.9;
      }

      this->MoveToNextGainIndex();
      current_case_ = 0;
      break;
    }
  }

  const double tolerance = 0.001;
  double sum = accumulate(gain_deltas_.begin(), gain_deltas_.end(), 0.0);

  return sum > tolerance;
}

std::vector<double> Twiddle::GainDeltas() {
  return gain_deltas_;
}
