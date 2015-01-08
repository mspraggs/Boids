#ifndef BOID_HPP
#define BOID_HPP

#include <iostream>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <utils.hpp>
#include <world.hpp>

namespace boids
{
  class Boid
  {
  public:

    typedef Eigen::Vector3d Coord;

    template <typename T>
    Boid(const T& x, const T& forward, const T& up, const double v_mag,
         const World* world, const double sight_range, const double min_dist,
         const double view_angle, const double align_max,
         const double cohese_max, const double separate_max);
    Boid(const Boid& boid);
    ~Boid() { };

    Boid& operator=(const Boid& rhs);

    void step_setup(const std::vector<Boid>& swarm);
    void step(const double dt);

    Coord& get_x() const { return this->x_; }
    Coord& get_forward() const { return this->forward_; }
    const double min_dist() const { return this->min_dist_; }
    const double sight_range() const { return this->sight_range_; }
    const double view_angle() const { return this->view_angle_; }

  private:
    const double point_heading(const double x, const double y) const;
    const double neighbour_distance(const Boid& boid) const;
    const double neighbour_heading(const Boid& boid) const {
      // Determine neighbour heading of specified boid
      return this->point_heading(boid._r_x, boid._r_y);
    }
    // Determines whether the supplied boid is in the fov of this boid
    const bool is_in_fov(const Boid& boid) const
    { return (this->neighbour_distance(boid) < this->_sight_range
              && fabs(this->neighbour_heading(boid)) < this->_view_angle); }
    const bool rightof(const double x, const double y) const;
    const bool rightof(const Boid& boid) const
    { return this->rightof(boid._r_x, boid._r_y); }
    const std::vector<int> get_neighbours(const std::vector<Boid>& swarm) const;
    const double v_mag() const
    { return math::magnitude(this->_v_x, this->_v_y); }
    // Angle between position vector and x-axis
    const double r_theta() const
    { return compute_phi(this->_r_x, this->_r_y); }

    Coord x_, v_;
    double v_mag_;
    World* world_;
    double sight_range_, min_dist_, view_angle_;
    double align_max_, cohese_max_, separate_max_;
    double dtheta_;
  };
}

#endif
