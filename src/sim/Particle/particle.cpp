#include "particle.hpp"
#include <cmath>
#include <iostream>
#include <unordered_map>

Particle::Particle(float mass) : mass(mass) {
  time = 0;
  velocity = 0;
  displacement = 0;
  prevDisplacement = 0;

  position = Vector2f::Zero();
  netForce = Vector2f::Zero();
}

void Particle::applyForce(Vector2f f) { netForce += f; }

void Particle::bindForce(functionOfTime f) { forces.push_back(f); }

void Particle::calcNetForce() {
  for (const auto &f : forces) {
    netForce += f(time);
  }
}

float Particle::calcAcceleration() { return netForce.norm() / mass; }

// Euler Integeration, for motion. Velocity... are public but this function will
// be used for rotation so needs to be generalised.
void Particle::implicitEulerIntegration(float &velocity, float &position,
                                        float acceleration, float dt) {
  // integrate a(t) twice to get displacement
  // Uses implicit euler integration
  if (acceleration == 0)
    return;

  velocity += acceleration * dt;
  position += velocity * dt;
}

void Particle::timeStep() {
  calcNetForce();
  acceleration = calcAcceleration();

  implicitEulerIntegration(velocity, displacement, acceleration, TIME_STEP);
  calculatePosition();

  netForce = Vector2f::Zero();
}

// calculates the position and direction currently moving in
void Particle::calculatePosition() {
  float changeInDistance = displacement - prevDisplacement;

  // moves position in same direction of force
  position += netForce.normalized() * changeInDistance;
  prevDisplacement = displacement;
}

Vector2f Particle::getPosition() { return position; }

Vector2f Particle::getVelocityVector() {
  return netForce.normalized() * velocity;
}
