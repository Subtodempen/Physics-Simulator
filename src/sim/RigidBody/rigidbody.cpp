#include "rigidbody.hpp"
#include <cmath>

RigidBody::RigidBody(float mass, Matrix2X verticies) : Particle(mass) {
  this->verticies = verticies;

  tourqe = 0;
  angularAcceleration = 0;
  angularVelocity = 0;
  orientation = 0;

  centre = numerical::calcCentre(verticies);
  inertialMoment = calcInertialMoment();
  // sort verticies into ascending order
}

void RigidBody::rbTimeStep() {
  timeStep();
  rotationalTimeStep();
}

void RigidBody::rotationalTimeStep() {
  angularAcceleration = calcAngularAcceleration();

  implicitEulerIntegration(angularVelocity, orientation, angularAcceleration,
                           TIME_STEP);

  tourqe = 0;
}

float RigidBody::calcInertialMoment() {
  auto triangles = numerical::triangulisePolygon(verticies, centre);
  float I = 0;

  for (const auto &triangle : triangles) {
    I += calcInertialMoment(triangle, centre);
  }

  return I;
}

// dont want to use the centre of our triangle which is what our formula yields
// us, so we calculate it with the formula for a typoicasl triangle, than use
// the parrelel axis theorm to find it at our polygons centre

float RigidBody::calcInertialMoment(Triangle triangle, Vector2f centre) {
  // inertial moment at triangles C is bh^3 / 36, use parrelel axis therom
  // to find the inertial mass at our polygons centre
  float b = 0, h = 0, I = 0;
  Vector2f midPoint = (triangle.A + triangle.C) / 2;

  b = (triangle.C - triangle.A).norm();
  h = (triangle.B - midPoint).norm();

  I = (mass * (pow(b, 2) + pow(h, 2))) / 36;

  // uses parrelel axis theporom
  Vector2f d = centre - numerical::calcCentre(triangle.getVertices());
  I += mass * pow(d.norm(), 2);

  return I;
}

void RigidBody::applyForce(Vector2f f, Vector2f position) {
  // update force
  netForce += f;

  // calculates tourque using r * F
  auto r = (position - centre);
  tourqe += (r.x() * f.x()) - (r.y() * f.y()); // finds the cross product
}

float RigidBody::calcAngularAcceleration() { return tourqe / inertialMoment; }
float RigidBody::getOrientation() { return orientation; }
