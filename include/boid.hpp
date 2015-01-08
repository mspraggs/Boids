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

    const double r_x() const { return this->_r_x; } 
    const double r_y() const { return this->_r_y; }
    // Angle between velocity vector and x-axis
    const double v_theta() const
    { return compute_phi(this->_v_x, this->_v_y); }
    const std::vector<double>& x_range() const { return this->_x_range; }
    const std::vector<double>& y_range() const { return this->_y_range; }
    const double x_span() const { return this->_x_span; }
    const double y_span() const { return this->_y_span; }
    const double min_dist() const { return this->_min_dist; }
    const double sight_range() const { return this->_sight_range; }
    const double view_angle() const { return this->_view_angle; }

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
    void correct_coordinates();
    const double correct_x(const double x) const
    { return correct_coord(x, this->_x_span); }
    const double correct_y(const double y) const
    { return correct_coord(y, this->_y_span); }
    const double v_mag() const
    { return math::magnitude(this->_v_x, this->_v_y); }
    // Angle between position vector and x-axis
    const double r_theta() const
    { return compute_phi(this->_r_x, this->_r_y); }

    double _r_x, _r_y, _v_x, _v_y, _v_mag;
    std::vector<double> _x_range, _y_range;
    double _x_span, _y_span;
    double _sight_range, _min_dist, _view_angle;
    double _align_max, _cohese_max, _separate_max;
    double _dtheta;
    int _index;
  };
}

#endif
