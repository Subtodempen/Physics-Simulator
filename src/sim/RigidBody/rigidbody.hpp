#include "../../maths/calculations.hpp"
#include "../Particle/particle.hpp"

class RigidBody : protected Particle {
public:
  RigidBody(float mass, Matrix2X verticies);

  void applyForce(Vector2f f, Vector2f position);

private:
  Matrix2X verticies;

  float angularAcceleration;
  float angularVelocity;
  float angle;

  Vector2f tourqe;
  float inertialMoment; // leave as global but change in future

  Vector2f centre;

  void calcInertialMoment();
  float calcInertialMoment(Triangle triangle, Vector2f centre);
};
