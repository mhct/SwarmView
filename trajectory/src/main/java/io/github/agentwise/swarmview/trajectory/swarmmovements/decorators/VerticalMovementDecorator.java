package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

public class VerticalMovementDecorator implements FiniteTrajectory4d {

  private final FiniteTrajectory4d trajectory4d;
  private final double initialHeight;
  private final double velocityZ;

  private VerticalMovementDecorator(
      FiniteTrajectory4d trajectory4d, double velocityZ) {
    this.trajectory4d = trajectory4d;
    this.initialHeight = trajectory4d.getDesiredPosition(0).z();
    this.velocityZ = velocityZ;
  }

  public static VerticalMovementDecorator create(
      FiniteTrajectory4d trajectory4d, double velocityZ) {
    return new VerticalMovementDecorator(trajectory4d, velocityZ);
  }

  @Override
  public double getTrajectoryDuration() {
    return trajectory4d.getTrajectoryDuration();
  }

  @Override
  public Pose getDesiredPosition(double timeInSeconds) {
    final Pose initialPose = trajectory4d.getDesiredPosition(timeInSeconds);
    final double decoratedZ = initialPose.z() + timeInSeconds * velocityZ;
    return Pose.create(initialPose.x(), initialPose.y(), decoratedZ, initialPose.yaw());
  }
}
