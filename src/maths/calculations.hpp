#pragma once

#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <functional>
#include <unordered_map>

using Matrix2X = Eigen::Matrix<float, Eigen::Dynamic, 2>;
using Vector2f = Eigen::Vector2f;

using functionOfTime = std::function<Vector2f(float)>;
// is a function of time returns a non-specific vector

// define a triangle ABC as a set of verticies
struct Triangle {
  Vector2f A, B, C;
};

namespace numerical {

float eulerIntegration(float startValueX, float startValueY, float approxIndex,
                       std::function<float(float)> dy, float h);

float eulerIntegrationMemo(float startValueX, float startValueY,
                           float approxIndex, std::function<float(float)> dy,
                           float h, std::unordered_map<float, float> &memo);

std::vector<Triangle> triangulisePolygon(Matrix2X verticies, Vector2f centre);

Vector2f calcCentre(Matrix2X verticies);
} // namespace numerical
