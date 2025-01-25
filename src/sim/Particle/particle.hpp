#pragma once

#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <functional>

#include "../../maths/calculations.hpp"

#define TIME_STEP 0.05

class Particle {
public:
  Particle(float mass);

  void applyForce(Vector2f f);
  void bindForce(functionOfTime f);
  void timeStep();

  Vector2f getPosition();
  Vector2f getVelocityVector();

protected:
  float mass;
  float time;

  Vector2f position;

  float displacement;
  float prevDisplacement;

  std::vector<functionOfTime> forces; // all forces acting upon an object
  Vector2f netForce;

  void implicitEulerIntegration(float &, float &, float, float);

private:
  float acceleration;
  float velocity;

  void calcNetForce();
  float calcAcceleration();

  float directionOfVector(const Vector2f &v);
  virtual void calculatePosition();
};
