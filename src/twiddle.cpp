#include "twiddle.h"
#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <numeric>      // std::accumulate
#include <limits>

using std::cout;
using std::endl;
using std::numeric_limits;
using std::vector;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double tol, const std::vector<double>& p, const std::vector<double>& dp, const std::string type) {
  is_initialized_ = false;
  tolerance_ = tol;
  p_ = p;
  dp_ = dp;
  type_ = type;

  p_index_ = 0;
  best_error_ = numeric_limits<double>::max();
  steps_ = 0;
  error_ = 0;
  state_ = State::S1;
  tune_ = true;
}

void Twiddle::UpdateError(double error) {
  if (!tune_) return;

  double sum_dp = std::accumulate(dp_.begin(), dp_.end(), 0.0);
  if (sum_dp < tolerance_) {
    tune_ = false;
    std::cout << "Type: " << type_ << " Kp: " << p_[0] << " Ki: " << p_[1] << " Kd: " << p_[2] << std::endl;
    return;
  }

  // error_ += error * error;
  error_ += pow(error, 2);

  if ((steps_ + 1) % kMaxSteps) {
    steps_ += 1;
    return;
  }

  if (!is_initialized_) {
    best_error_ = error_;
    is_initialized_ = true;
    return;
  }

  switch (state_) {
    case State::S1:
      p_[p_index_] += dp_[p_index_];
      state_ = State::S2;
      break;
    case State::S2:
      if (error_ < best_error_) {
        // Increasing param reduced error, so record and continue to increase dp range.
        best_error_ = error_;
        dp_[p_index_] *= 1.1;
        p_index_++;
        state_ = State::S1;
      } else {
        p_[p_index_] -= 2 * dp_[p_index_];
        state_ = State::S3;
      }
      break;
    case State::S3:
      if (error_ < best_error_) {
        // Increasing param reduced error, so record and continue to increase dp range.
        best_error_ = error_;
        dp_[p_index_] *= 1.1;
      } else {
        p_[p_index_] += dp_[p_index_];
        dp_[p_index_] *= 0.9;
      }
      p_index_++;
      state_ = State::S1;
      break;
  }

  steps_ = 0;
  error_ = 0;
  p_index_ %= p_.size();
}

vector<double> Twiddle::getParams() {
  return p_;
}

bool Twiddle::isDone() {
  return (tune_ == false);
}
