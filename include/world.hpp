/* World class that handles the world geometry and agent interaction with the
 * world
 */

#ifndef WORLD_HPP
#define WORLD_HPP

#include <Eigen/Dense>

#include <utils.hpp>

namespace boids {

  template <int D>
  class World
  {
  public:
    typedef Eigen::Vector<double, D, 1> Coord;

    // Constructor
    template <typename T1, typename T2>
    World(const T1& lower_corner, const T2& upper_corner);
    World(const World<D>& world);
    ~World() { };

    // Assignment operator
    World<D>& operator=(const World<D>& rhs);

    void correct_coord(Coord& coordinate) const;
    double point_separation(const Coord& coord1, const Coord coord2) const;
    
  private:
    Coord lower_corner_;
    Coord upper_corner_;
    Coord dimensions_;
  };



  template <int D>
  template <typename T1, typename T2>
  World<D>::World(const T1& lower_corner, const T2& upper_corner)
  {
    // Assign the lower and upper corners from the supplied array-like
    // variables

    for (int i = 0; i < D; ++i) {
      this->lower_corner_[i] = lower_corner[i];
      this->upper_corner_[i] = upper_corner[i];
    }
    this->dimensions_ = this->upper_corner_ - this->lower_corner_;
  }



  template <int D>
  World<D>::World(const World<D>& world)
  {
    // Copy constructor
    this->lower_corner_ = world.lower_corner_;
    this->upper_corner_ = world.upper_corner_;
    this->dimensions_ = world.dimensions_;
  }



  template <int D>
  World<D>& World<D>::operator=(const World<D>& rhs)
  {
    // Copy assignment operator
    if (&rhs != this) {
      this->lower_corner_ = rhs.lower_corner_;
      this->upper_corner_ = rhs.upper_corner_;
      this->dimensions_ = rhs.upper_corner_;
    }
    return *this;
  }



  template <int D>
  void World<D>::correct_coord(World<D>::Coord& coord) const
  {
    // Correct coordinate for the world geometry

    for (int i = 0; i < D; ++i) {
      coord[i] = math::mod((coord[i] + this->dimensions_[i] / 2),
                           this->dimensions_[i]) - this->dimensions_[i] / 2;
    }
  }



  template <int D>
  double World<D>::point_separation(const World<D>::Coord& coord1,
                                    const World<D>::Coord& coord2) const
  {
    // Determine the distance between the two supplied points
    Coord diff = coord2 - coord1;
    this->correct_coord(diff);
    return diff.norm();
  }
}
#endif
