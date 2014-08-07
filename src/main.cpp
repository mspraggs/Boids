#include <random>

#include <QtGui>

#include <boid.hpp>
#include <raster_window.hpp>

std::vector<boids::Boid> create_swarm()
{
  // RNG
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<> unidist(0.0, 1.0);

  // World parameters
  int num_boids = 100;
  std::vector<double> x_range{-0.7, 0.7};
  std::vector<double> y_range{-0.7, 0.7};
  double v_mag = 0.01;
  double sight_range = 0.15;
  double min_dist = 0.05;
  double view_angle = 135 * boids::math::pi / 180;
  double align_max = 5 * boids::math::pi / 180;
  double separate_max = 3 * boids::math::pi / 180;
  double cohese_max = 1.5 * boids::math::pi / 180;
  
  // Initialize boids
  std::vector<boids::Boid> swarm;
  for (int i = 0; i < num_boids; ++i) {
    double x = x_range[0] + (x_range[1] - x_range[0]) * unidist(generator);
    double y = y_range[0] + (y_range[1] - y_range[0]) * unidist(generator);
    double phi = 2 * boids::math::pi * unidist(generator) - boids::math::pi;
    swarm.push_back(boids::Boid(x, y, phi, v_mag, x_range, y_range, sight_range,
				min_dist, view_angle, align_max, cohese_max,
				separate_max, i));
  }

  return swarm;
}

int main(int argc, char* argv[])
{
  QGuiApplication app(argc, argv);
  RasterWindow win;
  win.show();
  return app.exec();
}
