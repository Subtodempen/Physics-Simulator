#include "rigidbody.hpp"

RigidBody::RigidBody(float mass, Matrix2X verticies) : Particle(mass) {
  this->verticies = verticies;

  centre = numerical::calcCentre(verticies);
}

// Finds the average vertex.
// Or middle, in an object with uniform mass this is the centre

void RigidBody::calcInertialMoment() {
  auto triangles = numerical::triangulisePolygon(verticies, centre);
  int I = 0;

  for (const auto &triangle : triangles) {
    I += calcInertialMoment(triangle, centre);
  }

  inertialMoment = I;
}

float RigidBody::calcInertialMoment(Triangle triangle, Vector2f centre) {
  // inertial moment at triangles C is bh^3 / 36, use parrelel axis therom
  // to find the inertial mass at our polygons centre
  float b = 0, h = 0, IC = 0;
  Vector2f midPoint = (triangle.B + triangle.C) / 2;

  b = (triangle.C - triangle.A).norm();
  h = (triangle.B - midPoint).norm();

  IC = (b * pow(h, 3)) / 36;

  // uses parrelel axis theporom
  return IC;
}

void RigidBody::applyForce(Vector2f f, Vector2f position) {
  // update force
  netForce += f;
  tourqe += (position - centre).cross(f); // calculates tourque using r * F
}
