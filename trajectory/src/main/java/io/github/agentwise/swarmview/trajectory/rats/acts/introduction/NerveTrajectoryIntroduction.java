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

    nerve.moveToPointWithVelocity(Point4D.create(3.0, 3.5, 2.0, yaw), 1.0);
    nerve.hover(3);
    nerve.rotateToAngle(yaw + 0.17 * 3, 0.5);
    nerve.hover(2);
    nerve.rotateToAngle(yaw - 0.17 * 3, 0.5);
    nerve.hover(1);
    nerve.rotateToAngle(yaw + 0.17 * 3, 0.5);
    nerve.rotateToAngle(yaw - 0.17 * 3, 0.5);
    nerve.hover(1);
    nerve.rotateToAngle(yaw + 0.17 * 3, 0.5);
    nerve.hover(2);
    nerve.rotateToAngle(yaw - 0.17 * 3, 0.5);
    nerve.rotateToAngle(yaw + 0.17 * 3, 0.5);
    nerve.rotateToAngle(yaw - 0.17 * 3, 0.5);
    nerve.moveNervouslyToPoint(
        Point4D.create(3.0, 3.5, 1.0, yaw), 0.3, 0.19, 1.0, 0.5, 2.8, 0.0, 1.0, 0.19, 2);
    nerve.moveToPointWithVelocity(Point4D.create(2, 1, 1.0, yaw), 1.0);

    // PHASE 3
    nerve.moveNervouslyToPoint(
        Point4D.create(2, 1, 3.2, yaw), 0.3, 0.19, 1.0, 1.0, 3.5, 0.19, 1.5, 0.19, 6);
//    nerve.rotateToAngle(yaw + 0.13 * 3, 2);
//    nerve.rotateToAngle(yaw + 0.10 * 3, 3);
//    nerve.rotateToAngle(yaw + -0.16 * 3, 4);
//    nerve.rotateToAngle(yaw + 0.08 * 3, 2);
//    nerve.rotateToAngle(yaw + -0.12 * 3, 3);
//    nerve.rotateToAngle(yaw + -0.15 * 3, 2.5);
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
