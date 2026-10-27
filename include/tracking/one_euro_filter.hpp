#ifndef TRACKING_ONE_EURO_FILTER_HPP
#define TRACKING_ONE_EURO_FILTER_HPP

class OneEuroFilter {
 public:
  OneEuroFilter(double minCutoff = 1.2, double beta = 0.01, double dCutoff = 1.0);

  void reset();
  double filter(double x, double dt);

 private:
  double alpha(double cutoff, double dt);

  double minCutoff_ = 1.2;
  double beta_ = 0.01;
  double dCutoff_ = 1.0;
  bool init_ = false;
  double xPrev_ = 0.0;
  double dxPrev_ = 0.0;
};

#endif
