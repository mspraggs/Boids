#ifndef SWARM_APPLICATION_HPP
#define SWARM_APPLICATION_HPP

#include <vector>

#include <QtGui>

#include "boid.hpp"

class SwarmApplication : public QGuiApplication
{

public:
  explicit SwarmApplication(
    std::vector<boids::Boid>& swarm,
    const boids::World& world,
    int& argc, char* argv[])
    : QGuiApplication(argc, argv), swarm_(swarm), world_(world)
  { self = this; }

  static SwarmApplication* instance()
  { return static_cast<SwarmApplication*>(self); }

  std::vector<boids::Boid>& swarm_;
  const boids::World& world_;

private:
  static SwarmApplication* self;

};

#endif
