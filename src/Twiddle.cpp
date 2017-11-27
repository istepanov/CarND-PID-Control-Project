#include "Twiddle.h"
#include <math.h>
#include <numeric>

Twiddle::Twiddle(PID& pid): pid_(pid)
{
    this->Reset();
}

Twiddle::~Twiddle() {}

void Twiddle::Reset() {
  gain_deltas_ = std::vector<double> {0.05, 0.0, 0.5};
  current_gain_index_ = 0;
  best_error_ = -1.0;
  current_case_ = 0;
}

double Twiddle::BestError() {
    return best_error_;
}

double Twiddle::GainDeltaSum() {
  return std::accumulate(gain_deltas_.begin(), gain_deltas_.end(), 0.0);
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

  return true;




  //double gain_delta_sum = std::accumulate(gain_deltas_.begin(), gain_deltas_.end(), 0.0);



    // p = [0, 0, 0]
    // dp = [1, 1, 1]
    // robot = make_robot()
    // x_trajectory, y_trajectory, best_err = run(robot, p)
    //
    // it = 0
    // while sum(dp) > tol:
    //     for i in range(len(p)):
    //         p[i] += dp[i]
    //         robot = make_robot()
    //         x_trajectory, y_trajectory, err = run(robot, p)
    //
    //         if err < best_err:
    //             best_err = err
    //             dp[i] *= 1.1
    //         else:
    //             p[i] -= 2 * dp[i]
    //             robot = make_robot()
    //             x_trajectory, y_trajectory, err = run(robot, p)
    //
    //             if err < best_err:
    //                 best_err = err
    //                 dp[i] *= 1.1
    //             else:
    //                 p[i] += dp[i]
    //                 dp[i] *= 0.9
    //     it += 1
    // return p
}
