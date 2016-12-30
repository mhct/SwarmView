package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/** @author Hoang Tung Dinh */
public final class VerticalCircleDecorator implements FiniteTrajectory4d {

  private final FiniteTrajectory4d trajectory;
  private final double radius;
  private final double initialPhase;
  private final double frequency;
  private final Point4D shift;

  private VerticalCircleDecorator(
      FiniteTrajectory4d trajectory,
      double radius,
      double initialPhase,
      double frequency,
      Point4D shift) {
    this.trajectory = trajectory;
    this.radius = radius;
    this.initialPhase = initialPhase;
    this.frequency = frequency;
    this.shift = shift;
  }

  public static VerticalCircleDecorator create(
      FiniteTrajectory4d trajectory,
      double radius,
      double initialPhase,
      double frequency,
      Point4D shift) {
    return new VerticalCircleDecorator(trajectory, radius, initialPhase, frequency, shift);
  }

  @Override
  public double getTrajectoryDuration() {
    return trajectory.getTrajectoryDuration();
  }

  @Override
  public Pose getDesiredPosition(double timeInSeconds) {
    final double circleShiftX =
        radius * StrictMath.sin(2 * StrictMath.PI * frequency * timeInSeconds + initialPhase);
    final double circleShiftZ =
        radius * StrictMath.cos(2 * StrictMath.PI * frequency * timeInSeconds + initialPhase);

    final Pose initialPose = trajectory.getDesiredPosition(timeInSeconds);
    final Pose decoratedCirclePose =
        Pose.create(
            initialPose.x() + circleShiftX,
            initialPose.y(),
            initialPose.z() + circleShiftZ,
            initialPose.yaw());
    final Pose decoratedCirclePoseWithShift =
        Pose.create(
            decoratedCirclePose.x() + shift.getX(),
            decoratedCirclePose.y() + shift.getY(),
            decoratedCirclePose.z() + shift.getZ(),
            decoratedCirclePose.yaw() + shift.getAngle());
    return decoratedCirclePoseWithShift;
  }
}
