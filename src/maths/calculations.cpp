#include "calculations.hpp"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <unordered_map>
using namespace numerical;

float numerical::eulerIntegrationMemo(float startValueX, float startValueY,
                                      float approxIndex,
                                      std::function<float(float)> dy, float h,
                                      std::unordered_map<float, float> &memo) {
  if (memo.find(approxIndex) != memo.end())
    return memo[approxIndex];

  memo[approxIndex] =
      eulerIntegration(startValueX, startValueY, approxIndex, dy, h);
  return memo[approxIndex];
}

float numerical::eulerIntegration(float startValueX, float startValueY,
                                  float approxIndex,
                                  std::function<float(float)> dy, float h) {
  float newY = (dy(startValueX) * h) + startValueY;
  float newX = startValueX + h;

  if (approxIndex == newX)
    return newY;

  else if (newX > approxIndex) {
    h = approxIndex - startValueX;
    return eulerIntegration(startValueX, startValueY, approxIndex, dy, h);
  }

  return eulerIntegration(newX, newY, approxIndex, dy, h);
}

float numerical::vectorAngle(Vector2f v1, Vector2f v2) {
  float dot = v1.dot(v2);
  float mag = v1.norm() * v2.norm();

  return acosf(dot / mag);
}

std::vector<Triangle> numerical::triangulisePolygon(Matrix2X verticies,
                                                    Vector2f centre) {
  std::vector<Triangle> triangles;

  for (int i = 0; i < verticies.rows(); i++) {
    Triangle t;

    t.B = centre;
    t.A = verticies.row(i);
    t.C = verticies.row((i + 1) % verticies.rows());

    triangles.push_back(t);
  }

  return triangles;
}

// find average verticies
// or when mass is uniform calcultes the centre point
Vector2f numerical::calcCentre(Matrix2X verticies) {
  Vector2f vertexSum = Vector2f::Zero();

  for (int i = 0; i < verticies.rows(); i++) {
    auto vertex = verticies.row(i);
    vertexSum += vertex;
  }

  return vertexSum / verticies.rows();
}

Triangle::Triangle() {
  A = Vector2f::Zero();
  B = Vector2f::Zero();
  C = Vector2f::Zero();
}

Matrix2X Triangle::getVertices() {
  Matrix2X matrix(3, 2);

  matrix.row(0) = A;
  matrix.row(1) = B;
  matrix.row(2) = C;

  return matrix;
}
