package io.github.agentwise.swarmview.trajectory.applications.trajectory;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;

import static com.google.common.base.Preconditions.checkArgument;
import static java.lang.StrictMath.atan;
import static java.lang.StrictMath.sin;

/**
 * Swing trajectory in 3D space as a 4D trajectory.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
class ShortSwingTrajectory4D extends SwingTrajectory4D {

  ShortSwingTrajectory4D(
      Point4D origin, double phase, double xzPlaneAngle, double radius, double frequency) {
    super(origin, phase, xzPlaneAngle, radius, frequency, true);
  }

  public static ShortSwingTrajectory4D create(
      Point4D begin, Point3D end, double height, double frequency) {
    Point4D origin = getSimpleSwingOriginFromPointsAndHeight(begin, end, height);
    double distance = Point3D.distance(Point3D.project(begin), end);
    double radius = calculateRadiusFrom(distance, height);
    double phase = getInitialPhaseFromHeightAndDistance(distance, height);
    Point4D normVector = Point4D.minus(Point4D.from(end, begin.getAngle()), begin);
    double xzPlaneAngle = StrictMath.atan2(normVector.getY(), normVector.getX());
    return new ShortSwingTrajectory4D(origin, phase, xzPlaneAngle, radius, frequency);
  }

  public static Point4D getSimpleSwingOriginFromPointsAndHeight(
      Point4D begin, Point3D end, double height) {
    checkArgument(
        begin.getZ() == end.getZ(),
        "Begin and end point should be equal in height for this to work.");
    double distance = Point3D.distance(Point3D.project(begin), end);
    checkArgument(
        distance > 2 * height, "The distance should be longer than the double of the drop height.");
    Point4D normVector = Point4D.minus(Point4D.from(end, begin.getAngle()), begin);
    Point4D center = begin.plus(Point4D.from(Point3D.scale(Point3D.project(normVector), 1 / 2d)));
    double radius = distance / 2 * sin(getInitialPhaseFromHeightAndDistance(distance, height));
    return center.plus(Point4D.create(0, 0, radius - height, 0));
  }

  /**
   * Used in short swing trajectory.
   *
   * @param distance
   * @param height
   * @return
   */
  public static double calculateRadiusFrom(double distance, double height) {
    return distance / 2 * sin(getInitialPhaseFromHeightAndDistance(distance, height));
  }

  public static double getInitialPhaseFromHeightAndDistance(double distance, double height) {
    checkArgument(distance > 0 && height > 0, "Distance and Height should be positive");

    return 2 * atan(2 * height / distance);
  }

  static ShortSwingBuilder ShortSwingBuilder() {
    return new ShortSwingBuilder();
  }

  /** Builder for 4D swing trajectories. */
  public static class ShortSwingBuilder {
    private Point4D begin;
    private Point3D end;
    private double heightDrop = 1;
    private double frequency;

    ShortSwingBuilder() {
      begin = Point4D.origin();
    }

    /**
     * Default value is Point3D.origin()
     *
     * @param location The origin of the circle.
     * @return this builder
     */
    public ShortSwingBuilder setBeginPoint(Point4D origin) {
      this.begin = origin;
      return this;
    }

    /**
     * The end point of the swing movement. Should have the same height as begin point.
     *
     * @param location The origin of the circle.
     * @return this builder
     */
    public ShortSwingBuilder setEndPoint(Point3D end) {
      this.end = end;
      return this;
    }

    /**
     * Default frequency = 0.
     *
     * @param frequency The frequency of the circle.
     * @return this builder
     */
    public ShortSwingBuilder setFrequency(double frequency) {
      this.frequency = frequency;
      return this;
    }

    /**
     * Set the height to drop in meters along the trajectory. Mind that this drop height should be
     * smaller than double the distance between begin and end to work.
     *
     * @param heightDrop
     * @return
     */
    public ShortSwingBuilder setDropHeight(double heightDrop) {
      this.heightDrop = heightDrop;
      return this;
    }

    /** @return A new trajectory instance that represents a swing motion. */
    public ShortSwingTrajectory4D build() {
      return ShortSwingTrajectory4D.create(begin, end, heightDrop, frequency);
    }
  }
}
