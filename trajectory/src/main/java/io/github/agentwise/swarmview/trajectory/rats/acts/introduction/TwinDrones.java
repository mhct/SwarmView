package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.VerticalCircleDecorator;

public final class TwinDrones {
  private static final double YAW = -StrictMath.PI / 2;

  private TwinDrones() {}

  public static FiniteTrajectory4d createRomeoTrajectory(
      Pose initialPose, Pose finalPose, double startTime) {
    final FiniteTrajectory4d commonTrajectory =
        getCommonTrajectory(initialPose, finalPose, startTime);
    return VerticalCircleDecorator.create(
        commonTrajectory, 0.5, 0, 0.15, Point4D.create(0, 0, 0, 0), startTime);
  }

  public static FiniteTrajectory4d createJulietTrajectory(
      Pose initialPose, Pose finalPose, double startTime) {
    final FiniteTrajectory4d commonTrajectory =
        getCommonTrajectory(initialPose, finalPose, startTime);
    return VerticalCircleDecorator.create(
        commonTrajectory, 0.5, StrictMath.PI, 0.15, Point4D.create(0, -1.0, 0, 0), startTime);
  }

  private static FiniteTrajectory4d getCommonTrajectory(
      Pose initialPose, Pose finalPose, double startTime) {
    final Particle drone = new Particle(initialPose);

    if (startTime > 0) {
      drone.hover(startTime);
    }

    drone.moveToPointWithVelocity(Point4D.create(4, 2, 2, YAW), 1);
    drone.moveVerticalCorkscrew(Point4D.create(4.5, 2.5, 2, YAW), 0.1, 3, 10);
    drone.moveVerticalCorkscrew(Point4D.create(4.5, 2.5, 2, YAW), 0.1, 1.5, 10);
    drone.moveToPointWithVelocity(Point4D.create(2, 2, 2, YAW), 1);
    drone.moveVerticalCorkscrew(Point4D.create(2.5, 2.5, 2, YAW), 0.1, 3, 10);
    drone.moveVerticalCorkscrew(Point4D.create(2.5, 2.5, 2, YAW), 0.1, 1.5, 10);
    drone.moveToPointWithVelocity(Point4D.create(5.5, 3.3, 2, YAW), 1);
    drone.moveVerticalCorkscrew(Point4D.create(6, 3.8, 2, YAW), 0.1, 1.5, 10);
//    drone.moveVerticalCorkscrew(Point4D.create(6, 3.8, 2, YAW), 0.1, 1.5, 10);
    drone.moveToPointWithVelocity(Point4D.from(finalPose), 2);

    return drone.getTrajectory();
  }
}
