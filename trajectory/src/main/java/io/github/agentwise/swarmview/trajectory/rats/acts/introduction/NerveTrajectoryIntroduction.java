/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;

/** @author tom */
public class NerveTrajectoryIntroduction implements FiniteTrajectory4d {

  private final Particle nerve;

  public NerveTrajectoryIntroduction(Pose initialPosition, Pose finalPosition, double start)
      throws Exception {
    final double yaw = -Math.PI / 2;

    nerve = new Particle(initialPosition);
    if (start > 0) {
      nerve.hover(start);
    }
    // PHASE 1
    nerve.moveToPointWithVelocity(Point4D.create(6, 3, 1.0, yaw), 1.5);
    nerve.moveNervouslyToPoint(
        Point4D.create(3.5, 3.55, 1.8, yaw), 0.3, 0.19, 1.0, 0.8, 1.8, 0.19, 1.5, 0.19, 20);
    // PHASE 2
    nerve.rotateToAngle(yaw + 0.17 * 5, 3);
    nerve.rotateToAngle(yaw + -0.17 * 5, 3);
    nerve.rotateToAngle(yaw + 0.17 * 5, 3);
    nerve.rotateToAngle(yaw + -0.17 * 5, 3);
    nerve.moveNervouslyToPoint(
        Point4D.create(2, 0, 3.2, yaw), 0.3, 0.19, 1.0, 1.5, 3.2, 0.19, 1.5, 0.19, 20);
    // PHASE 3
    nerve.moveNervouslyToPoint(
        Point4D.create(2, 0, 3.2, yaw), 0.3, 0.19, 1.0, 1.0, 3.5, 0.19, 1.5, 0.19, 10);
    nerve.moveToPointWithVelocity(
        Point4D.create(finalPosition.x(), finalPosition.y(), finalPosition.z(), yaw), 0.5);
  }

  @Override
  public double getTrajectoryDuration() {
    return nerve.getTrajectory().getTrajectoryDuration();
  }

  @Override
  public Pose getDesiredPosition(double timeInSeconds) {
    return nerve.getTrajectory().getDesiredPosition(timeInSeconds);
  }
}
