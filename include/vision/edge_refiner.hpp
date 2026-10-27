#ifndef VISION_EDGE_REFINER_HPP
#define VISION_EDGE_REFINER_HPP

#include "core/types.hpp"

bool refineQuad(const cv::Mat& gray, Quad& q, int searchRadius);

#endif
