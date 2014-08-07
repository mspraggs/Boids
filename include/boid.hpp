#ifndef BOID_HPP
#define BOID_HPP

#include <iostream>
#include <cmath>
#include <vector>

#include <utils.hpp>
#include <swarm.hpp>

namespace boids
{
  class Boid
  {
  public:
    Boid(const double x, const double y, const double heading,
	 const double v_mag, const std::vector<double>& x_range,
	 const std::vector<double>& y_range, const double sight_range,
	 const double min_dist, const double view_angle, const double align_max,
	 const double cohese_max, const double separate_max, const int index);
    Boid(const Boid& boid);
    ~Boid();

    Boid& operator=(const Boid& rhs);

    void step_setup(const std::vector<Boid>& swarm);
    void step(const double dt);

  private:
    const double point_heading(const double x, const double y) const;
    const double neighbour_distance(const Boid& boid) const;
    const double neighbour_heading(const Boid& boid) const;
    const bool is_in_fov(const Boid& boid) const;
    const bool rightof(const double x, const double y) const;
    const bool rightof(const Boid& boid) const;
    const std::vector<int> get_neighbours(const std::vector<Boid>& swarm) const;
    void correct_coordinates();
    const double correct_x(const double x) const;
    const double correct_y(const double y) const;
    const double v_mag() const
    { return math::magnitude(this->_v_x, this->_v_y); }
    const double v_theta() const;
    const double r_theta() const;

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
