package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/** @author Hoang Tung Dinh */
public class HorizontalCircleDecorator implements FiniteTrajectory4d {
  private final FiniteTrajectory4d trajectory;
  private final Point3D center;
  private final double frequency;

  private HorizontalCircleDecorator(
      FiniteTrajectory4d trajectory, Point3D center, double frequency) {
    this.trajectory = trajectory;
    this.center = Point3D.create(center.getX(), center.getY(), 0);
    this.frequency = frequency;
  }

  public static HorizontalCircleDecorator create(
      FiniteTrajectory4d trajectory, Point3D center, double frequency) {
    return new HorizontalCircleDecorator(trajectory, center, frequency);
  }

  @Override
  public double getTrajectoryDuration() {
    return trajectory.getTrajectoryDuration();
  }

  @Override
  public Pose getDesiredPosition(double timeInSeconds) {
    final Pose initialPose = trajectory.getDesiredPosition(timeInSeconds);
    final double radius =
        Point3D.distance(Point3D.create(initialPose.x(), initialPose.y(), 0), center);
    final double shiftX = radius * StrictMath.sin(2 * StrictMath.PI * frequency * timeInSeconds);
    final double shiftY = radius * StrictMath.cos(2 * StrictMath.PI * frequency * timeInSeconds);
    final Pose decoratedPose =
        Pose.create(
            initialPose.x() + shiftX, initialPose.y() + shiftY, initialPose.z(), initialPose.yaw());
    return decoratedPose;
  }
}
