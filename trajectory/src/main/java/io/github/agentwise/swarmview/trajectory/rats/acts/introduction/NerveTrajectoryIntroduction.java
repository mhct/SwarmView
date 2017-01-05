/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;

/** @author tom */
public class NerveTrajectoryIntroduction implements FiniteTrajectory4d {

  private final Particle nerve;

  public NerveTrajectoryIntroduction(Pose initialPosition, Pose finalPosition,
      double start) throws Exception {
    final double yaw = -Math.PI / 2;

    nerve = new Particle(initialPosition);
    if (start > 0) {
      nerve.hover(start);
    }

//    nerve.moveToPointWithVelocity(Point4D.create(3.0, 3.5, 2.0, yaw), 1.0);
    nerve.hover(10);
    nerve.rotateToAngle(yaw + 0.71, 0.2);
    nerve.hover(4);
    nerve.rotateToAngle(yaw - 0.60, 0.2);
    nerve.hover(3);
//    nerve.moveToPointWithVelocity(Point4D.create(3.0, 3.5, 2.5, yaw), 1.0);
//    nerve.hover(3);
//    nerve.rotateToAngle(yaw + 0.40, 0.2);
//    nerve.hover(2);
//    nerve.rotateToAngle(yaw - 0.51, 0.2);
//    nerve.hover(2);
//    nerve.rotateToAngle(yaw + 0.37, 0.2);
//    nerve.hover(3);
//    nerve.rotateToAngle(yaw - 0.60, 0.2);
//    nerve.hover(2);
//    nerve.rotateToAngle(yaw + 0.72, 0.2);
//    nerve.hover(2);
//    nerve.rotateToAngle(yaw - 0.80, 0.2);

    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.1, yaw + 0.7), 2.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.2, yaw + 0.6), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.3, yaw + 0.5), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.4, yaw + 0.4), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.5, yaw + 0.3), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.6, yaw + 0.2), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.7, yaw - 0.8), 2.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.8, yaw), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 1.9, yaw + 0.5), 2.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.0, yaw + 0.4), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.1, yaw + 0.3), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.2, yaw + 0.2), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.3, yaw), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.4, yaw - 0.2), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.5, yaw - 0.4), 1.0);
    nerve.moveToPoint(Point4D.create(3.0, 3.5, 2.6, yaw - 0.6), 1.0);

    nerve.hover(3);
    nerve.moveNervouslyToPoint(Point4D.create(3.0, 3.5, 1.0, yaw), 0.3, 0.19, 1.0, 1, 3.5, 0.0, 2,
        0.19, 8);
    nerve.moveToPointWithVelocity(Point4D.create(2, 1, 1.0, yaw), 1.0);

    // PHASE 3
    nerve.moveNervouslyToPoint(Point4D.create(2, 1, 3.2, yaw), 0.3, 0.19, 1.2, 1.0, 3.5, 0.19, 1.5,
        0.19, 6);
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
