#include "tracking/one_euro_filter.hpp"

#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

OneEuroFilter::OneEuroFilter(double minCutoff, double beta, double dCutoff)
    : minCutoff_(minCutoff), beta_(beta), dCutoff_(dCutoff) {}

void OneEuroFilter::reset() {
  init_ = false;
  xPrev_ = 0.0;
  dxPrev_ = 0.0;
}

double OneEuroFilter::alpha(double cutoff, double dt) {
  double tau = 1.0 / (2.0 * kPi * cutoff);
  return 1.0 / (1.0 + tau / dt);
}

double OneEuroFilter::filter(double x, double dt) {
  if (!init_ || dt <= 0) {
    init_ = true;
    xPrev_ = x;
    dxPrev_ = 0.0;
    return x;
  }
  double dx = (x - xPrev_) / dt;
  dxPrev_ += alpha(dCutoff_, dt) * (dx - dxPrev_);
  double cutoff = minCutoff_ + beta_ * std::abs(dxPrev_);
  xPrev_ += alpha(cutoff, dt) * (x - xPrev_);
  return xPrev_;
}
