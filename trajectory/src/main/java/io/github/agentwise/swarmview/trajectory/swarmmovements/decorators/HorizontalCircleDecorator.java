package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/** @author Hoang Tung Dinh */
public class HorizontalCircleDecorator implements FiniteTrajectory4d {
  private final FiniteTrajectory4d trajectory;
  private final Point3D center;
  private final double frequency;
  private final double phase;
  private final double startTime;

  private HorizontalCircleDecorator(
      FiniteTrajectory4d trajectory, Point3D center, double frequency, double startTime) {
    this.trajectory = trajectory;
    this.center = Point3D.create(center.getX(), center.getY(), 0);
    this.frequency = frequency;
    this.startTime = startTime;
    final Pose firstPose = trajectory.getDesiredPosition(startTime);
    final double dx = firstPose.x() - center.getX();
    final double dy = firstPose.y() - center.getY();
    phase = StrictMath.atan2(dx, dy);
  }

  public static HorizontalCircleDecorator create(
      FiniteTrajectory4d trajectory, Point3D center, double frequency, double startTime) {
    return new HorizontalCircleDecorator(trajectory, center, frequency, startTime);
  }

  @Override
  public double getTrajectoryDuration() {
    return trajectory.getTrajectoryDuration();
  }

  @Override
  public Pose getDesiredPosition(double timeInSeconds) {
    if (timeInSeconds >= startTime) {
      final Pose initialPose = trajectory.getDesiredPosition(timeInSeconds);
      final double radius =
          Point3D.distance(Point3D.create(initialPose.x(), initialPose.y(), 0), center);
      final double shiftX =
          radius * StrictMath.sin(2 * StrictMath.PI * frequency * (timeInSeconds - startTime) + phase);
      final double shiftY =
          radius * StrictMath.cos(2 * StrictMath.PI * frequency * (timeInSeconds - startTime) + phase);
      final Pose decoratedPose =
          Pose.create(
              center.getX() + shiftX,
              center.getY() + shiftY,
              initialPose.z(),
              initialPose.yaw());
      return decoratedPose;
    } else {
      return trajectory.getDesiredPosition(timeInSeconds);
    }
  }
}
