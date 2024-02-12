#pragma once

#include <cmath>

namespace InputUtils {
typedef struct {
  double x;
  double y;
  bool deadzoneApplied;
} DeadzoneAxes;

DeadzoneAxes CalculateCircularDeadzone(double x, double y, double deadzoneDistance) {
  if (std::hypot(x, y) > deadzoneDistance) {
    return {.x = x, .y = y, .deadzoneApplied = false};
  }

  return {.x = 0, .y = 0, .deadzoneApplied = true};
}
}  // namespace InputUtils