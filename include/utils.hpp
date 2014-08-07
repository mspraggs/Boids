#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>

namespace boids
{
  namespace math
  {
    const double pi = 3.14159265358979323846;
    
    const double mod(const double n, const double m);
    
    const double magnitude(const double x, const double y);

    const int sgn(const double x);
  }

  const double correct_coord(const double coord, const double span);

  const double compute_phi(const double x, const double y);
}

#endif
