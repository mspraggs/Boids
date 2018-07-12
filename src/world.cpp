#include "utils.hpp"
#include "world.hpp"

namespace boids {

  World::World(
    const World::Coord& lower_corner,
    const World::Coord& upper_corner)
  : lower_corner_(lower_corner), upper_corner_(upper_corner),
  dimensions_(upper_corner - lower_corner)
  {
    // Constructor
  }



  World::World(const World& world)
  {
    // Copy constructor
    this->lower_corner_ = world.lower_corner_;
    this->upper_corner_ = world.upper_corner_;
    this->dimensions_ = world.dimensions_;
  }



  World& World::operator=(const World& rhs)
  {
    // Copy assignment operator
    if (&rhs != this) {
      this->lower_corner_ = rhs.lower_corner_;
      this->upper_corner_ = rhs.upper_corner_;
      this->dimensions_ = rhs.upper_corner_;
    }
    return *this;
  }



  void World::correct_coord(World::Coord& coord) const
  {
    // Correct coordinate for the world geometry

    for (int i = 0; i < 3; ++i) {
      coord[i] = math::mod((coord[i] + this->dimensions_[i] / 2),
                           this->dimensions_[i]) - this->dimensions_[i] / 2;
    }
  }



  World::Coord World::point_diff(const World::Coord& coord1,
                                 const World::Coord& coord2) const
  {
    // Determine the difference between two points
    Coord diff = coord2 - coord1;
    this->correct_coord(diff);
    return diff;
  }



  double World::point_separation(const World::Coord& coord1,
                                 const World::Coord& coord2) const
  {
    // Determine the distance between the two supplied points
    return this->point_diff(coord1, coord2).norm();
  }
}
