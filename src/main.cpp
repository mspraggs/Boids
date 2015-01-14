#include <random>

#include <QtGui>

#include <boid.hpp>
#include <raster_window.hpp>
#include <swarm_application.hpp>
#include <world.hpp>

SwarmApplication* SwarmApplication::self = 0;

std::vector<boids::Boid> create_swarm(const boids::World& world)
{
  // RNG
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<> unidist(0.0, 1.0);

  // World parameters
  int num_boids = 100;
#ifdef _MSC_VER
  double range[] = {-200.0, 200.0};
  std::vector<double> x_range(range, range + 2);
  std::vector<double> y_range(range, range + 2);
#else
  std::vector<double> x_range{-200.0, 200.0};
  std::vector<double> y_range{-200.0, 200.0};
#endif
  double v_mag = 100.0;
  double sight_range = 20.0;
  double min_dist = 10.0;
  double view_angle = 135 * boids::math::pi / 180;
  double align_max = 5 * boids::math::pi / 180;
  double separate_max = 3 * boids::math::pi / 180;
  double cohese_max = 1.5 * boids::math::pi / 180;
  
  // Initialize boids
  std::vector<boids::Boid> swarm;
  boids::Boid::Coord x_axis;
  x_axis << 1.0, 0.0, 0.0;
  boids::Boid::Coord up;
  up << 0.0, 0.0, 1.0;
  for (int i = 0; i < num_boids; ++i) {
    boids::Boid::Coord random_coord
      = world.get_lower_corner()
        + boids::Boid::Coord::Random().cwiseProduct(world.get_dimensions());
    random_coord[2] = 0.0;
    double phi = 2 * boids::math::pi * unidist(generator) - boids::math::pi;
    boids::Boid::Coord forward = Eigen::AngleAxisd(phi, up) * x_axis;
    swarm.push_back(boids::Boid(random_coord, forward, up, v_mag, &world, sight_range,
                                min_dist, view_angle, align_max, cohese_max,
                                separate_max));
  }

  return swarm;
}

int main(int argc, char* argv[])
{
  boids::World world(-200.0 * boids::Boid::Coord::Ones(),
                     200.0 * boids::Boid::Coord::Ones());
  std::vector<boids::Boid> swarm = create_swarm(world);

  SwarmApplication app(swarm, world, argc, argv);
  RasterWindow win;
  win.show();
  return app.exec();
}
