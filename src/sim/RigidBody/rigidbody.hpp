#include "../../maths/calculations.hpp"
#include "../Particle/particle.hpp"

#include <cmath>
#include <iostream>

class RigidBody : public Particle {
public:
  RigidBody(float mass, Matrix2X verticies);
  void rbTimeStep();

  void applyForce(Vector2f f, Vector2f position);
  float getOrientation();

private:
  Matrix2X verticies;

  float angularAcceleration;
  float angularVelocity;
  float orientation;

  float tourqe;
  float inertialMoment;

  Vector2f centre;

  void rotationalTimeStep();

  float calcInertialMoment();
  float calcInertialMoment(Triangle triangle, Vector2f centre);

  float calcAngularAcceleration();
};
