/* World class that handles the world geometry and agent interaction with the
 * world
 */

#ifndef WORLD_HPP
#define WORLD_HPP

#include <Eigen/Dense>

#include <utils.hpp>

namespace boids {

  class World
  {
  public:
    typedef Eigen::Vector3d Coord;
    // Constructor
    template <typename T1, typename T2>
    World(const T1& lower_corner, const T2& upper_corner);
    World(const World& world);
    ~World() { };

    // Assignment operator
    World& operator=(const World& rhs);

    void correct_coord(Coord& coordinate) const;
    Coord point_diff(const Coord& coord1, const Coord& coord2) const;
    double point_separation(const Coord& coord1, const Coord coord2) const;
    double max_distance() const
    { return (this->upper_corner_ - this->lower_corner_).norm() / 2; }
    
  private:
    Coord lower_corner_;
    Coord upper_corner_;
    Coord dimensions_;
  };
}
#endif
