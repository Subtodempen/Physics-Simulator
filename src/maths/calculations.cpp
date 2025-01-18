#include "calculations.hpp"
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

std::vector<Triangle> numerical::triangulisePolygon(Matrix2X verticies,
                                                    Vector2f centre) {
  std::vector<Triangle> triangles;

  for (int i = 0; i < verticies.rows() - 1; i++) {
    Triangle t;

    t.B = centre;
    t.A = verticies.row(i);
    t.C = verticies.row(i + 1);

    triangles.push_back(t);
  }

  return triangles;
}

Vector2f numerical::calcCentre(Matrix2X verticies) {
  Vector2f vertexSum = Vector2f::Zero();

  for (int i = 0; i < verticies.rows(); i++) {
    auto vertex = verticies.row(i);
    vertexSum += vertex;
  }

  return vertexSum / verticies.rows();
}
