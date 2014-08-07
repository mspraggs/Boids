#include <boid.hpp>

namespace boids
{
  Boid::Boid(const double x, const double y, const double heading,
	     const double v_mag, const std::vector<double>& x_range,
	     const std::vector<double>& y_range, const double sight_range,
	     const double min_dist, const double view_angle,
	     const double align_max, const double cohese_max,
	     const double separate_max, const int index)
    : _r_x(x), _r_y(y), _v_mag(v_mag), _x_range(x_range), _y_range(y_range),
      _sight_range(sight_range), _min_dist(min_dist), _view_angle(view_angle),
      _align_max(align_max), _cohese_max(cohese_max),
    _separate_max(separate_max), _index(index)
  {
    this->_x_span = x_range[1] - x_range[0];
    this->_y_span = y_range[1] - y_range[0];
    this->_v_x = cos(heading) * v_mag;
    this->_v_y = sin(heading) * v_mag;
  }


  
  Boid::Boid(const Boid& boid)
    : _r_x(boid._r_x), _r_y(boid._r_y), _v_x(boid._v_x), _v_y(boid._v_y), 
      _v_mag(boid._v_mag), _x_range(boid._x_range), _y_range(boid._y_range),
      _x_span(boid._x_span), _y_span(boid._y_span),
      _sight_range(boid._sight_range), _min_dist(boid._min_dist),
      _view_angle(boid._view_angle), _align_max(boid._align_max),
      _cohese_max(boid._cohese_max), _separate_max(boid._separate_max),
      _index(boid._index)
  {
    // Copy constructor
  }



  Boid::~Boid()
  {
    // Empty destructor
  }



  Boid& Boid::operator=(const Boid& rhs)
  {
    // Copy assignment operator
    if (&rhs != this) {
      this->_r_x = rhs._r_x;
      this->_r_y = rhs._r_y;
      this->_v_x = rhs._v_x;
      this->_v_y = rhs._v_y;
      this->_v_mag = rhs._v_mag;
      this->_x_range = rhs._x_range;
      this->_y_range = rhs._y_range;
      this->_sight_range = rhs._sight_range;
      this->_min_dist = rhs._min_dist;
      this->_view_angle = rhs._view_angle;
      this->_align_max = rhs._align_max;
      this->_cohese_max = rhs._cohese_max;
      this->_separate_max = rhs._separate_max;
      this->_index = rhs._index;
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
	double current_neighbour_distance = this->neighbour_distance(swarm[i]);
	if (current_neighbour_distance < min_dist) {
	  nearest_neighbour = i;
	  min_dist = current_neighbour_distance;
	}
      }

      if (min_dist < this->_min_dist) {
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
	double neighbour_heading
	  = swarm[nearest_neighbour].v_theta() - this->v_theta();
	this->_dtheta
	  = (this->_separate_max < fabs(neighbour_heading))
	  ? math::sgn(neighbour_heading) * this->_separate_max
	  : neighbour_heading;
      }
    }
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



  const double Boid::point_heading(const double x, const double y) const
  {
    // Determines the heading of the specified coordinate
    double dx = this->correct_x(x - this->_r_x);
    double dy = this->correct_y(y - this->_r_y);
    double dot_prod = dx * this->_v_x + dy * this->_v_y;
    double dr = math::magnitude(dx, dy);
    if (dr < 1e-10) return 0.0;
    double cos_alpha = dot_prod / (dr * this->_v_mag);
    if (this->rightof(x, y)) return acos(cos_alpha);
    else return -acos(cos_alpha);
  }



  const double Boid::neighbour_distance(const Boid& boid) const
  {
    // Determines the distance to the specified neighbour
    double dx = this->correct_x(boid._r_x - this->_r_x);
    double dy = this->correct_y(boid._r_y - this->_r_y);
    return math::magnitude(dx, dy);
  }



  const double Boid::neighbour_heading(const Boid& boid) const
  {
    // Determines the heading of the specified boid
    return this->point_heading(boid._r_x, boid._r_y);
  }



  const bool Boid::is_in_fov(const Boid& boid) const
  {
    // Determines whether the supplied boid is in the fov of this boid
    return (this->neighbour_distance(boid) < this->_sight_range
	    && fabs(this->neighbour_heading(boid) < this->_view_angle));
  }



  const bool Boid::rightof(const double x, const double y) const
  {
    // Determines whether the supplied boid is right of the current boid
    // or not
    double dx = this->correct_x(this->_r_x - x);
    double dy = this->correct_y(this->_r_y - y);
    return this->_v_x * dy - this->_v_y * dx > 0;
  }



  const bool Boid::rightof(const Boid& boid) const
  {
    // Determines whether the supplied boid is right of the current boid
    // or not
    return this->rightof(boid._r_x, boid._r_y);
  }



  const std::vector<int>
  Boid::get_neighbours(const std::vector<Boid>& swarm) const
  {
    // Returns a list of indices of neighbours
    std::vector<int> neighbours;
    for (unsigned int i = 0; i < swarm.size(); ++i)
      if (this->is_in_fov(swarm[i]) && swarm[i]._index != this->_index)
	neighbours.push_back(i);
    return neighbours;
  }

  
  
  void Boid::correct_coordinates()
  {
    // Adjusts the coordinates of the boid to account for the wrap-around nature
    // of the world.
    this->_r_x = this->correct_x(this->_r_x);
    this->_r_y = this->correct_y(this->_r_y);
  }

  

  const double Boid::correct_x(const double x) const
  {
    // Adjusts the supplied x component in accordance with the wrap-around
    // geometry
    return correct_coord(x, this->_x_span);
  }



  const double Boid::correct_y(const double y) const
  {
    // Adjusts the supplied y component in accordance with the wrap-around
    // geometry
    return correct_coord(y, this->_y_span);
  }



  const double Boid::v_theta() const
  {
    // Compute the angle sub-tended by the velocity vector and the x-axis
    return compute_phi(this->_v_x, this->_v_y);
  }



  const double Boid::r_theta() const
  {
    // Compute the angle sub-tended by the position vector and the x-axis
    return compute_phi(this->_r_x, this->_r_y);
  }
}
