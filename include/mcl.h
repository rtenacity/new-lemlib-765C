#ifndef __MCL_H__
#define __MCL_H__

#include "main.h"
#include "config.h"
#include "chassis.h"
#include <random>


struct Particle {
  lemlib::Pose pose; // Use the lemlib::Pose for x, y, theta
  float weight;      // Weight of the particle (probability)

  // Constructor - required since lemlib::Pose has no default constructor
  Particle(const lemlib::Pose &p = lemlib::Pose(0, 0, 0), float w = 0.0f)
      : pose(p), weight(w) {}
};

namespace MCL {

}

#endif