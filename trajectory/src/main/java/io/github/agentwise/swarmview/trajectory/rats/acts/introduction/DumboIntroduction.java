/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;

/** @author tom */
public class DumboIntroduction {

  private final Particle dumbo;
  private static final double YAW = -StrictMath.PI / 2;

  public DumboIntroduction(Pose initialPose, Pose finalPose, double start) {

    dumbo = new Particle(initialPose);
    if (start > 0) {
      dumbo.hover(start);
    }
    final double percentageVelocity = 5.0;
    dumbo.wiggle(2, 1);
    dumbo.moveZigZagToPoint(Point4D.create(1, 3, 2.8, YAW), percentageVelocity);
    dumbo.wiggle(3, 1);
    dumbo.moveZigZagToPoint(Point4D.create(1, 4.5, 1, YAW), percentageVelocity);
    dumbo.wiggle(3, 1);
    dumbo.moveZigZagToPoint(Point4D.create(5, 3.0, 2, YAW), percentageVelocity);
    dumbo.wiggle(2, 1);
    dumbo.moveZigZagToPoint(Point4D.create(5, 2, 3.5, YAW), percentageVelocity);
    dumbo.wiggle(2, 1);
    dumbo.moveZigZagToPoint(Point4D.create(4, 3.0, 1.0, YAW), percentageVelocity);
    dumbo.wiggle(2, 1);
    dumbo.moveZigZagToPoint(Point4D.from(finalPose), percentageVelocity);
  }

  public FiniteTrajectory4d getTrajectory() {
    return dumbo.getTrajectory();
  }

  public static FiniteTrajectory4d createTrajectory(
      Pose initialPosition, Pose finalPosition, double startTime) {

    DumboIntroduction dumbo = new DumboIntroduction(initialPosition, finalPosition, startTime);
    return dumbo.getTrajectory();
  }
}
