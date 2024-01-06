#include "utils/SwerveUtils.h"

#include <cmath>
#include <numbers>

double SwerveUtils::StepTowards(double _current, double _target,
                                double _stepsize) {
  if (abs(_current - _target) <= _stepsize) {
    return _target;
  } else if (_target < _current) {
    return _current - _stepsize;
  } else {
    return _current + _stepsize;
  }
}

double SwerveUtils::StepTowardsCircular(double _current, double _target,
                                        double _stepsize) {
  _current = WrapAngle(_current);
  _target = WrapAngle(_target);

  double temp = _target - _current;
  double stepDirection = temp > 0 ? 1 : temp < 0 ? -1 : 0;  // Get sign
  double difference = abs(_current - _target);

  if (difference <= _stepsize) {
    return _target;
  } else if (difference > std::numbers::pi) {  // does the system need to wrap
                                               // over eventually?
    // handle the special case where you can reach the target in one step while
    // also wrapping
    if (_current + 2 * std::numbers::pi - _target < _stepsize ||
        _target + 2 * std::numbers::pi - _current < _stepsize) {
      return _target;
    } else {
      return WrapAngle(_current -
                       stepDirection *
                           _stepsize);  // this will handle wrapping gracefully
    }

  } else {
    return _current + stepDirection * _stepsize;
  }
}

double SwerveUtils::AngleDifference(double _angleA, double _angleB) {
  double difference = abs(_angleA - _angleB);
  return difference > std::numbers::pi ? (2 * std::numbers::pi) - difference
                                       : difference;
}

double SwerveUtils::WrapAngle(double _angle) {
  double twoPi = 2 * std::numbers::pi;

  if (_angle ==
      twoPi) {  // Handle this case separately to avoid floating point errors
                // with the floor after the division in the case below
    return 0.0;
  } else if (_angle > twoPi) {
    return _angle - twoPi * floor(_angle / twoPi);
  } else if (_angle < 0.0) {
    return _angle + twoPi * (floor((-_angle) / twoPi) + 1);
  } else {
    return _angle;
  }
}