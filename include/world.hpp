/* World class that handles the world geometry and agent interaction with the
 * world
 */

#ifndef WORLD_HPP
#define WORLD_HPP

#include <Eigen/Dense>


namespace boids {

  class World
  {
  public:
    typedef Eigen::Vector3d Coord;
    // Constructor
    World(const Coord& lower_corner, const Coord& upper_corner);
    World(const World& world);
    ~World() { };

    // Assignment operator
    World& operator=(const World& rhs);

    const Coord& get_lower_corner() const { return lower_corner_; }
    const Coord& get_upper_corner() const { return upper_corner_; }
    const Coord& get_dimensions() const {return dimensions_; }

    void correct_coord(Coord& coordinate) const;
    Coord point_diff(const Coord& coord1, const Coord& coord2) const;
    double point_separation(const Coord& coord1, const Coord& coord2) const;
    double max_distance() const
    { return (this->upper_corner_ - this->lower_corner_).norm() / 2; }
    
  private:
    Coord lower_corner_;
    Coord upper_corner_;
    Coord dimensions_;
  };
}
#endif
