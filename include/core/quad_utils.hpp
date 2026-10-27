#ifndef CORE_QUAD_UTILS_HPP
#define CORE_QUAD_UTILS_HPP

#include "core/types.hpp"

double polyArea(const Quad& q);
double quadDist(const Quad& a, const Quad& b);
Quad sortClockwise(Quad pts);
Quad alignToReference(const Quad& candidate, const Quad& reference);

#endif
