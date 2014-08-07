#include <utils.hpp>

namespace boids
{
  namespace math
  {
    const double mod(const double n, const double m)
    {
      return fmod(fmod(n, m) + m, m);
    }



    const double magnitude(const double x, const double y)
    {
      return sqrt(x * x + y * y);
    }



    const int sgn(const double x)
    {
      return (x < 0) ? -1 : 1;
    }
  }



  const double correct_coord(const double coord, const double span)
  {
    return math::mod((coord + span / 2), span) - span / 2;
  }



  const double compute_phi(const double x, const double y)
  {
    double phi = atan(fabs(y / x));
    if (x > 0) {
      if (y > 0)
	return phi;
      else
	return -phi;
    }
    else {
      if (y > 0)
	return math::pi - phi;
      else
	return -(math::pi - phi);
    }
  }
}
