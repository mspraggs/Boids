#include <utils.hpp>

namespace boids {
  namespace math {
    double mod(const double n, const double m) {
      return fmod(fmod(n, m) + m, m);
    }


    double magnitude(const double x, const double y) {
      return sqrt(x * x + y * y);
    }


    int sgn(const double x) {
      return (x < 0) ? -1 : 1;
    }
  }
}
