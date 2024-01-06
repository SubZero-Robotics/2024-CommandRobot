class SwerveUtils {
 public:
  /**
   * Steps a value towards a target with a specified step size.
   *
   * @param _current The current or starting value.  Can be positive or
   * negative.
   * @param _target The target value the algorithm will step towards.  Can be
   * positive or negative.
   * @param _stepsize The maximum step size that can be taken.
   * @return The new value for {@code _current} after performing the specified
   * step towards the specified target.
   */
  static double StepTowards(double _current, double _target, double _stepsize);

  /**
   * Steps a value (angle) towards a target (angle) taking the shortest path
   * with a specified step size.
   *
   * @param _current The current or starting angle (in radians).  Can lie
   * outside the 0 to 2*PI range.
   * @param _target The target angle (in radians) the algorithm will step
   * towards.  Can lie outside the 0 to 2*PI range.
   * @param _stepsize The maximum step size that can be taken (in radians).
   * @return The new angle (in radians) for {@code _current} after performing
   * the specified step towards the specified target. This value will always lie
   * in the range 0 to 2*PI (exclusive).
   */
  static double StepTowardsCircular(double _current, double _target,
                                    double _stepsize);

  /**
   * Finds the (unsigned) minimum difference between two angles including
   * calculating across 0.
   *
   * @param _angleA An angle (in radians).
   * @param _angleB An angle (in radians).
   * @return The (unsigned) minimum difference between the two angles (in
   * radians).
   */
  static double AngleDifference(double _angleA, double _angleB);

  /**
   * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
   *
   * @param _angle The angle (in radians) to wrap.  Can be positive or negative
   * and can lie multiple wraps outside the output range.
   * @return An angle (in radians) from 0 and 2*PI (exclusive).
   */
  static double WrapAngle(double _angle);
};
