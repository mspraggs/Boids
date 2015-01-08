#include <world.hpp>

namespace boids {
  template <typename T1, typename T2>
  World::World(const T1& lower_corner, const T2& upper_corner)
  {
    // Assign the lower and upper corners from the supplied array-like
    // variables

    for (int i = 0; i < D; ++i) {
      this->lower_corner_[i] = lower_corner[i];
      this->upper_corner_[i] = upper_corner[i];
    }
    this->dimensions_ = this->upper_corner_ - this->lower_corner_;
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

    for (int i = 0; i < D; ++i) {
      coord[i] = math::mod((coord[i] + this->dimensions_[i] / 2),
                           this->dimensions_[i]) - this->dimensions_[i] / 2;
    }
  }



  double World::point_separation(const World::Coord& coord1,
                                 const World::Coord& coord2) const
  {
    // Determine the distance between the two supplied points
    Coord diff = coord2 - coord1;
    this->correct_coord(diff);
    return diff.norm();
  }
}
