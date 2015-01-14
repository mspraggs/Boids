#include <boid.hpp>

namespace boids
{

  Boid::Boid(
    const Boid::Coord& x, const Boid::Coord& forward, const Boid::Coord& up,
    const double v_mag, const World* world, const double sight_range,
    const double min_dist, const double view_angle, const double align_max,
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
      separate_max_(boid.separate_max_)
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
      double min_dist = this->world_->max_distance();
      for (int i : neighbours) {
        if (this->neighbour_distance(swarm[i]) < min_dist) {
          nearest_neighbour = i;
          min_dist = this->neighbour_distance(swarm[i]);
        }
      }
      
      if (min_dist > this->min_dist_) {
        Coord net_dx = Coord::Zero(); // Net neighbour orientatation
        Coord avg_x = Coord::Zero(); // Net neighbour coordinates
        
        for (int i : neighbours) {
          net_dx += swarm[i].forward_;
          avg_x += swarm[i].x_;
        }
        double cosphi = this->forward_.dot(net_dx) / net_dx.norm() / this->forward_.norm();
        double net_phi;
        if (cosphi > 1.0) {
          net_phi = 0.0;
        }
        else if (cosphi < -1.0) {
          net_phi = math::pi;
        }
        else {
          net_phi = acos(cosphi);
        }
        avg_x /= neighbours.size();
        double avg_phi = this->point_phi(avg_x);
        double alignment
          = (this->align_max_ < net_phi) ? this->align_max_ : net_phi;
        double cohesion
          = (this->cohese_max_ < avg_phi) ? this->cohese_max_ : avg_phi;
        this->step_matrix_
          = Eigen::AngleAxisd(cohesion,
                              this->forward_.cross(avg_x).normalized())
          * Eigen::AngleAxisd(alignment,
                              this->forward_.cross(net_dx).normalized());
      }
      else {
        double neighbour_phi = this->neighbour_phi(swarm[nearest_neighbour]);
        double separate
          = (this->separate_max_ < neighbour_phi)
            ? this->separate_max_ : neighbour_phi;
        Coord neighbour_vector = swarm[nearest_neighbour].x_ - this->x_;
        neighbour_vector.normalize();
        this->step_matrix_
          = Eigen::AngleAxisd(separate,
                              -this->forward_.cross(neighbour_vector));
      }
    }
    else
      this->step_matrix_ = Eigen::Matrix3d::Identity();
  }



  void Boid::step(const double dt)
  {
    this->forward_ = this->step_matrix_ * this->forward_;
    this->up_ = this->step_matrix_ * this->up_;
    this->forward_.normalize();
    this->up_.normalize();
    this->x_ += this->forward_ * this->v_mag_ * dt;
    this->world_->correct_coord(this->x_);
  }



  const double Boid::point_phi(const Boid::Coord& x) const
  {
    // Determines the angle phi of point x in spherical coordinate system
    // where direction of motion of this (forward_) is z-axis.
    Boid::Coord diff = this->world_->point_diff(x, this->x_);
    double dot_prod = this->forward_.dot(diff);
    double dr = diff.norm();
    if (dr < 1e-10) return 0.0;
    double cos_alpha = dot_prod / (dr * this->v_mag_);
    return acos(cos_alpha);
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
