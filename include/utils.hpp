#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>

#include <Eigen/Dense>

#include <boid.hpp>


namespace boids
{
  namespace math
  {
    const double pi = 3.14159265358979323846;
    
    double mod(const double n, const double m);
    
    double magnitude(const double x, const double y);

    int sgn(const double x);
  }
}

#endif
