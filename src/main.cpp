#include "maths/calculations.hpp"
#include "sim/Particle/particle.hpp"
#include "sim/RigidBody/rigidbody.hpp"
#include <iostream>
#include <numbers> // std::numbers
#include <ostream>
#include <vector>

int main() {
  /*
  Particle particle(1);

  const auto gravitationalForce = [](float t) {
    Vector2f g(0, -9.8);
    return g;
  };

  auto dragForce = [&](float t) -> Vector2f {
    float p = 1.2041; // Denisty of air
    float cd = 0.47;
    float A = (0.5 * 0.5) * std::numbers::pi; // area of our sphere d = 1

    Vector2f v = particle.getVelocityVector();
    Vector2f vUnit = v.normalized();

    return -0.5 * p * cd * A * (v.norm() * v.norm()) * vUnit;
  };

  particle.bindForce(gravitationalForce);
  particle.bindForce(dragForce);

  for (int i = 0; i < 100; i++) {
    particle.timeStep();
  }

  std::cout << particle.getPosition();
  */

  Matrix2X square(4, 2);
  square << 0, 0, 0, 3, 3, 3, 3, 0;

  RigidBody rb(10, square);

  int i = 0;
  while (i < 100) {
    rb.rbTimeStep();
    rb.applyForce({0, -9.8}, {1.7, 1.0}); // todo make sure in bounds of rb
    i++;
  }

  std::cout << rb.getOrientation() << std::endl << std::endl;
  std::cout << rb.getPosition();
}

/*
    Figure out what the forces are on an object
    Add those forces up to get a single “resultant” or “net” force
    Use F = ma to calculate the object’s acceleration due to those forces
    Use the object’s acceleration to calculate the object’s velocity
    Use the object’s velocity to calculate the object’s position
    Since the forces on the object may change from moment to moment, repeat this
   process from #1, forever.
*/
