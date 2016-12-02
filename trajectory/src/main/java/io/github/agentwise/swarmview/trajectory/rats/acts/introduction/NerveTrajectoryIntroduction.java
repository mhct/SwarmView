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
    final double heigh1 = 1.8;
    final double low1 = 0.8;
    final double x1 = 6;
    final double y1 = 3;
    final double speed1 = 1.5;

    final double heigh2 = 3.2;
    final double low2 = 1.5;
    final double x2 = 3.5;
    final double y2 = 3.55;
    final double speed2 = 1.5;

    final double heigh3 = 3.5;
    final double low3 = 1.0;
    final double x3 = 2;
    final double y3 = 0;
    final double speed3 = 1.5;

    nerve = new Particle(initialPosition);
    if (start > 0) {
      nerve.hover(start);
    }
    // PHASE 1
    nerve.moveToPointWithVelocity(Point4D.create(6, 3, 1.0, yaw), 1.5);
    nerve.moveNervouslyToPoint(
        Point4D.create(3.5, 3.55, 3.2, yaw), 0.3, 0.19, 1.0, 0.8, 1.8, 0.19, 1.5, 0.19, 20);
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
