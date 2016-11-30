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
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.1 + -0.06, y1 + 0.0, heigh1 + 0.04, yaw + 0.07 * 5), speed1 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.2 + 0.21, y1 + 0.02, low1 + 0.07, yaw + 0.09 * 5), speed1 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.3 + -0.08, y1 + 0.02, heigh1 + 0.07, yaw + -0.17 * 5), speed1 + -0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.4 + 0.21, y1 + 0.01, low1 + 0.0, yaw + -0.08 * 5), speed1 + 0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.5 + 0.02, y1 + -0.07, heigh1 + -0.11, yaw + 0.11 * 5), speed1 + 0.0);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.6 + 0.14, y1 + -0.1, low1 + -0.03, yaw + -0.02 * 5), speed1 + 0.02);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.7 + -0.05, y1 + -0.12, heigh1 + 0.17, yaw + 0.15 * 5), speed1 + -0.16);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.8 + -0.09, y1 + -0.07, low1 + 0.08, yaw + -0.15 * 5), speed1 + -0.14);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 0.9 + 0.22, y1 + -0.06, heigh1 + -0.06, yaw + 0.04 * 5), speed1 + 0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.0 + 0.03, y1 + 0.19, low1 + 0.19, yaw + -0.04 * 5), speed1 + 0.19);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.1 + -0.26, y1 + 0.0, heigh1 + 0.01, yaw + 0.03 * 5), speed1 + 0.08);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.2 + -0.09, y1 + -0.19, low1 + -0.11, yaw + -0.17 * 5), speed1 + -0.05);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.3 + 0.28, y1 + 0.03, heigh1 + 0.15, yaw + -0.12 * 5), speed1 + 0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.4 + -0.01, y1 + 0.15, low1 + -0.14, yaw + 0.05 * 5), speed1 + 0.02);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.5 + -0.02, y1 + -0.12, heigh1 + -0.12, yaw + -0.06 * 5), speed1 + 0.01);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.6 + -0.26, y1 + -0.11, low1 + -0.06, yaw + 0.08 * 5), speed1 + 0.06);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.7 + 0.09, y1 + -0.12, heigh1 + -0.14, yaw + 0.0 * 5), speed1 + 0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.8 + 0.1, y1 + 0.01, low1 + -0.01, yaw + 0.19 * 5), speed1 + -0.04);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 1.9 + 0.12, y1 + -0.13, heigh1 + 0.15, yaw + -0.04 * 5), speed1 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 2.0 + 0.03, y1 + -0.11, low1 + 0.18, yaw + 0.09 * 5), speed1 + -0.11);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 2.1 + -0.28, y1 + 0.0, heigh1 + 0.07, yaw + -0.05 * 5), speed1 + -0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 2.2 + 0.29, y1 + -0.02, low1 + 0.06, yaw + -0.02 * 5), speed1 + 0.01);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 2.3 + -0.28, y1 + 0.13, heigh1 + 0.09, yaw + 0.15 * 5), speed1 + -0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 2.4 + -0.19, y1 + -0.12, low1 + 0.04, yaw + 0.06 * 5), speed1 + 0.06);
		nerve.moveToPointWithVelocity(Point4D.create(x1 - 2.5 + 0.04, y1 + -0.2, heigh1 + 0.02, yaw + -0.02 * 5), speed1 + 0.04);
		// PHASE 2
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.0 + -0.06, y2 - 0 + 0.0, heigh2 + 0.04, yaw + 0.07 * 5), speed2 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.1 + 0.21, y2 - 0.15 + 0.02, low2 + 0.07, yaw + 0.09 * 5), speed2 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.2 + -0.08, y2 - 0.30 + 0.02, heigh2 + 0.07, yaw + -0.17 * 5), speed2 + -0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.3 + 0.21, y2 - 0.45 + 0.01, low2 + 0.0, yaw + -0.08 * 5), speed2 + 0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.4 + 0.02, y2 - 0.60 + -0.07, heigh2 + -0.11, yaw + 0.11 * 5), speed2 + 0.0);
		nerve.rotateToAngle(yaw +  0.17 * 5, 3);
		nerve.rotateToAngle(yaw +  -0.17 * 5, 3);
		nerve.rotateToAngle(yaw +  0.17 * 5, 3);
		nerve.rotateToAngle(yaw +  -0.17 * 5, 3);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.5 + 0.14, y2 - 0.75 + -0.1, low2 + -0.03, yaw + -0.02 * 5), speed2 + 0.02);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.6 + -0.05, y2 - 0.90 + -0.12, heigh2 + 0.17, yaw + 0.15 * 5), speed2 + -0.16);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.7 + -0.09, y2 - 1.05 + -0.07, low2 + 0.08, yaw + -0.15 * 5), speed2 + -0.14);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.8 + 0.22, y2 - 1.20 + -0.06, heigh2 + -0.06, yaw + 0.04 * 5), speed2 + 0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 0.9 + 0.03, y2 - 1.35 + 0.19, low2 + 0.19, yaw + -0.04 * 5), speed2 + 0.19);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.0 + -0.26, y2 - 1.50 + 0.0, heigh2 + 0.01, yaw + 0.03 * 5), speed2 + 0.08);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.1 + -0.09, y2 - 1.65 + -0.19, low2 + -0.11, yaw + -0.17 * 5), speed2 + -0.05);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.2 + 0.28, y2 - 1.80 + 0.03, heigh2 + 0.15, yaw + -0.12 * 5), speed2 + 0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.3 + -0.01, y2 - 1.95 + 0.15, low2 + -0.14, yaw + 0.05 * 5), speed2 + 0.02);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.4 + -0.02, y2 - 2.10 + -0.12, heigh2 + -0.12, yaw + -0.06 * 5), speed2 + 0.01);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + -0.26, y2 - 2.25 + -0.11, low2 + -0.06, yaw + 0.08 * 5), speed2 + 0.06);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.6 + 0.09, y2 - 2.40 + -0.12, heigh2 + -0.14, yaw + 0.0 * 5), speed2 + 0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + 0.1, y2 - 2.55 + 0.01, low2 + -0.01, yaw + 0.19 * 5), speed2 + -0.04);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + 0.12, y2 - 2.70 + -0.13, heigh2 + 0.15, yaw + -0.04 * 5), speed2 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + 0.03, y2 - 2.85 + -0.11, low2 + 0.18, yaw + 0.09 * 5), speed2 + -0.11);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + -0.28, y2 - 3.00 + 0.0, heigh2 + 0.07, yaw + -0.05 * 5), speed2 + -0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + 0.29, y2 - 3.15 + -0.02, low2 + 0.06, yaw + -0.02 * 5), speed2 + 0.01);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + -0.28, y2 - 3.30 + 0.13, heigh2 + 0.09, yaw + 0.15 * 5), speed2 + -0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 1.5 + -0.19, y2 - 3.45 + -0.12, low2 + 0.04, yaw + 0.06 * 5), speed2 + 0.06);
		nerve.moveToPointWithVelocity(Point4D.create(x2 - 2.4 + 0.04, y2 - 3.55 + -0.2, heigh2 + 0.02, yaw + -0.02 * 5), speed2 + 0.04);
		// PHASE 3
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.06, y3 + 0.0, heigh3 + 0.04, yaw + 0.07 * 5), speed3 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.21, y3 + 0.02, low3 + 0.07, yaw + 0.09 * 5), speed3 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.08, y3 + 0.02, heigh3 + 0.07, yaw + -0.17 * 5), speed3 + -0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.21, y3 + 0.01, low3 + 0.0, yaw + -0.08 * 5), speed3 + 0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.02, y3 + -0.07, heigh3 + -0.11, yaw + 0.11 * 5), speed3 + 0.0);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.14, y3 + -0.1, low3 + -0.03, yaw + -0.02 * 5), speed3 + 0.02);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.05, y3 + -0.12, heigh3 + 0.17, yaw + 0.15 * 5), speed3 + -0.16);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.09, y3 + -0.07, low3 + 0.08, yaw + -0.15 * 5), speed3 + -0.14);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.22, y3 + -0.06, heigh3 + -0.06, yaw + 0.04 * 5), speed3 + 0.18);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.03, y3 + 0.19, low3 + 0.19, yaw + -0.04 * 5), speed3 + 0.19);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.26, y3 + 0.0, heigh3 + 0.01, yaw + 0.03 * 5), speed3 + 0.08);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.09, y3 + -0.19, low3 + -0.11, yaw + -0.17 * 5), speed3 + -0.05);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.28, y3 + 0.03, heigh3 + 0.15, yaw + -0.12 * 5), speed3 + 0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.01, y3 + 0.15, low3 + -0.14, yaw + 0.05 * 5), speed3 + 0.02);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.02, y3 + -0.12, heigh3 + -0.12, yaw + -0.06 * 5), speed3 + 0.01);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.26, y3 + -0.11, low3 + -0.06, yaw + 0.08 * 5), speed3 + 0.06);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.09, y3 + -0.12, heigh3 + -0.14, yaw + 0.0 * 5), speed3 + 0.09);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.1, y3 + 0.01, low3 + -0.01, yaw + 0.19 * 5), speed3 + -0.04);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.12, y3 + -0.13, heigh3 + 0.15, yaw + -0.04 * 5), speed3 + 0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.03, y3 + -0.11, low3 + 0.18, yaw + 0.09 * 5), speed3 + -0.11);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.28, y3 + 0.0, heigh3 + 0.07, yaw + -0.05 * 5), speed3 + -0.12);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + 0.29, y3 + -0.02, low3 + 0.06, yaw + -0.02 * 5), speed3 + 0.01);
		nerve.moveToPointWithVelocity(Point4D.create(x3 + -0.28, y3 + 0.13, heigh3 + 0.09, yaw + 0.15 * 5), speed3 + -0.18);
		nerve.moveToPointWithVelocity(Point4D.create(finalPosition.x(), finalPosition.y(), finalPosition.z(), yaw), 0.5);
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
