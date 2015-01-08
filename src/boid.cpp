#include <boid.hpp>

namespace boids
{  
  template <typename T>
  Boid::Boid(const T& x, const T& forward, const T& up, const double v_mag,
             const World* world, const double sight_range, const double min_dist,
             const double view_angle, const double align_max,
             const double cohese_max, const double separate_max)
    : x_(x), v_mag_(v_mag), world_(world), sight_range_(sight_range),
      min_dist_(min_dist), view_angle_(view_angle), align_max_(align_max),
      cohese_max_(cohese_max), separate_max_(separate_max)
  {
    // Assign and normalize boid orientation
    this->forward_ = forward;
    this->forward_.normalize();
    this->up_ = up;
    this->up_.normalize();
    if (this->forward_.dot(this->up_) < 1e-8) {
      Boid::Coord right = this->forward_.cross(this->up_);
      this->up_ = right.cross(this->forward_);
      this->up_.normalize();
    }
  }


  
  Boid::Boid(const Boid& boid)
    : x_(boid.x_), forward_(boid.forward_), up_(boid.up_), v_mag_(boid.v_mag_),
      world_(boid.world_), sight_range_(boid.sight_range_),
      min_dist_(boid.min_dist_), view_angle_(boid.view_angle_),
      align_max_(boid.align_max_), cohese_max_(boid.cohese_max_),
      separate_max_(boid.separate_max_),
  {
    // Copy constructor
  }



  Boid& Boid::operator=(const Boid& rhs)
  {
    // Copy assignment operator
    if (&rhs != this) {
      this->x_ = rhs.x_;
      this->forward_ = rhs.forward_;
      this->up_ = rhs.up_;
      this->v_mag_ = rhs.v_mag_;
      this->world_ = rhs.world_;
      this->sight_range_ = rhs.sight_range_;
      this->min_dist_ = rhs.min_dist_;
      this->view_angle_ = rhs.view_angle_;
      this->align_max_ = rhs.align_max_;
      this->cohese_max_ = rhs.cohese_max_;
      this->separate_max_ = rhs.separate_max_;
    }
    return *this;
  }



  void Boid::step_setup(const std::vector<Boid>& swarm)
  {
    // Check for nearby neighbours and updates the position of the boid
    // accordingly
    std::vector<int> neighbours = this->get_neighbours(swarm);
    
    if (neighbours.size() > 0) {
      int nearest_neighbour = -1;
      double min_dist = this->_x_span + this->_y_span;
      for (int i : neighbours) {
        if (this->neighbour_distance(swarm[i]) < min_dist) {
          nearest_neighbour = i;
          min_dist = this->neighbour_distance(swarm[i]);
        }
      }
      
      if (min_dist > this->_min_dist) {
        double net_dx = 0.0;
        double net_dy = 0.0;
        double avg_x = 0.0;
        double avg_y = 0.0;
        
        for (int i : neighbours) {
          net_dx += swarm[i]._v_x / swarm[i].v_mag();
          net_dy += swarm[i]._v_y / swarm[i].v_mag();
          avg_x += swarm[i]._r_x;
          avg_y += swarm[i]._r_y;
        }
        double phi = compute_phi(net_dx, net_dy) - this->v_theta();
        avg_x /= neighbours.size();
        avg_y /= neighbours.size();
        double avg_heading = this->point_heading(avg_x, avg_y);
        double alignment
          = (this->_align_max < fabs(phi))
          ? math::sgn(phi) * this->_align_max
          : phi;
        double cohesion
          = (this->_cohese_max < fabs(avg_heading))
          ? math::sgn(avg_heading) * this->_cohese_max
          : avg_heading;
        this->_dtheta = cohesion + alignment;
      }
      else {
        double neighbour_phi
          = this->neighbour_phi(swarm[nearest_neighbour]);
        this->_dtheta
          = (this->_separate_max < fabs(neighbour_phi))
          ? math::sgn(neighbour_phi) * this->_separate_max
          : neighbour_phi;
      }
    }
    else
      this->_dtheta = 0.0;
  }



  void Boid::step(const double dt)
  {
    this->_v_x
      = cos(this->_dtheta) * this->_v_x
      - sin(this->_dtheta) * this->_v_y;
    this->_v_y
      = sin(this->_dtheta) * this->_v_x
      + cos(this->_dtheta) * this->_v_y;
    this->_v_x *= this->_v_mag / this->v_mag();
    this->_v_y *= this->_v_mag / this->v_mag();
    this->_r_x += this->_v_x * dt;
    this->_r_y += this->_v_y * dt;
    this->correct_coordinates();
  }



  const double Boid::point_phi(const Boid::Coord& x) const
  {
    // Determines the angle phi of point x in spherical coordinate system
    // where direction of motion of this (forward_) is z-axis.
    Boid::Coord diff = this->world_->point_diff(x, this->x_);
    double dot_prod = this->forward_.dot(diff);
    double dr = diff.norm();
    if (dr < 1e-10) return 0.0;
    double cos_alpha = dot_prod / (dr * this->_v_mag);
    return acos(cos_alpha);
  }



  const double Boid::neighbour_distance(const Boid& boid) const
  {
    // Determines the distance to the specified neighbour
    double dx = this->correct_x(boid._r_x - this->_r_x);
    double dy = this->correct_y(boid._r_y - this->_r_y);
    return math::magnitude(dx, dy);
  }



  const bool Boid::rightof(const double x, const double y) const
  {
    // Determines whether the supplied boid is right of the current boid
    // or not
    double dx = this->correct_x(this->_r_x - x);
    double dy = this->correct_y(this->_r_y - y);
    return this->_v_x * dy - this->_v_y * dx > 0;
  }



  const std::vector<int>
  Boid::get_neighbours(const std::vector<Boid>& swarm) const
  {
    // Returns a list of indices of neighbours
    std::vector<int> neighbours;
    for (unsigned int i = 0; i < swarm.size(); ++i)
      if (this->is_in_fov(swarm[i]) && &swarm[i] != this)
        neighbours.push_back(i);
    return neighbours;
  }
}
