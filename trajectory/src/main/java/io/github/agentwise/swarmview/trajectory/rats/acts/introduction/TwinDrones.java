package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.VerticalCircleDecorator;

public final class TwinDrones {
  private static final double YAW = -StrictMath.PI / 2;

  private static final Pose INITIAL_POSE = Pose.create(1, 2, 1.5, -StrictMath.PI / 2);
  private static final Pose FINAL_POSE = Pose.create(5, 4, 1.5, -StrictMath.PI / 2);

  private TwinDrones() {}

  public static FiniteTrajectory4d createRomeoTrajectory(
      Pose initialPose, Pose finalPose, double startTime) {
    final FiniteTrajectory4d commonTrajectory = getCommonTrajectory();
    final FiniteTrajectory4d decoratedTrajectory =
        VerticalCircleDecorator.create(commonTrajectory, 0.5, 0, 0.15, Point4D.create(0, 0, 0, 0));

    final Particle dummy = new Particle(initialPose);
    dummy.moveToPointWithVelocity(Point4D.from(decoratedTrajectory.getDesiredPosition(0)), 0.5);

    final Particle romeo = new Particle(initialPose);
    final double hoverTime = startTime - dummy.getTrajectory().getTrajectoryDuration();
    if (hoverTime > 0) {
      romeo.hover(hoverTime);
    }
    romeo.moveToPointWithVelocity(Point4D.from(decoratedTrajectory.getDesiredPosition(0)), 0.5);
    romeo.addMovement(decoratedTrajectory);
    romeo.moveToPointWithVelocity(Point4D.from(finalPose), 0.8);
    return romeo.getTrajectory();
  }

  public static FiniteTrajectory4d createJulietTrajectory(
      Pose initialPose, Pose finalPose, double startTime) {
    final FiniteTrajectory4d commonTrajectory = getCommonTrajectory();
    final FiniteTrajectory4d decoratedTrajectory =
        VerticalCircleDecorator.create(
            commonTrajectory, 0.5, StrictMath.PI, 0.15, Point4D.create(0, -1.0, 0, 0));

    final Particle dummy = new Particle(initialPose);
    dummy.moveToPointWithVelocity(Point4D.from(decoratedTrajectory.getDesiredPosition(0)), 0.5);

    final Particle juliet = new Particle(initialPose);
    final double hoverTime = startTime - dummy.getTrajectory().getTrajectoryDuration();
    if (hoverTime > 0) {
      juliet.hover(hoverTime);
    }
    juliet.moveToPointWithVelocity(Point4D.from(decoratedTrajectory.getDesiredPosition(0)), 0.5);
    juliet.addMovement(decoratedTrajectory);
    juliet.moveToPointWithVelocity(Point4D.from(finalPose), 0.8);
    return juliet.getTrajectory();
  }

  private static FiniteTrajectory4d getCommonTrajectory() {
    final Particle drone = new Particle(INITIAL_POSE);

    drone.moveToPointWithVelocity(Point4D.create(4, 3.5, 2, YAW), 0.3);
    drone.moveToPointWithVelocity(Point4D.create(1, 3.5, 2, YAW), 0.3);
    drone.moveToPointWithVelocity(Point4D.create(4, 3.5, 2, YAW), 0.3);
    drone.moveToPointWithVelocity(Point4D.create(1, 3.5, 2, YAW), 0.3);
    //    drone.moveVerticalCorkscrew(Point4D.create(4.0, 3.0, 2, YAW), 0.1, 3, 10);
    //    drone.moveVerticalCorkscrew(Point4D.create(4.0, 3.0, 2, YAW), 0.1, 1.5, 10);
    //    drone.moveToPointWithVelocity(Point4D.create(2, 2, 2, YAW), 1);
    //    drone.moveVerticalCorkscrew(Point4D.create(2.5, 3.0, 2, YAW), 0.1, 3, 10);
    //    drone.moveVerticalCorkscrew(Point4D.create(2.5, 3.0, 2, YAW), 0.1, 1.5, 10);
    //    drone.moveToPointWithVelocity(Point4D.create(4.5, 3.3, 2, YAW), 1);
    //    drone.moveVerticalCorkscrew(Point4D.create(5, 3.8, 2, YAW), 0.1, 1.5, 10);
    //    drone.moveVerticalCorkscrew(Point4D.create(6, 3.8, 2, YAW), 0.1, 1.5, 10);
    drone.moveToPointWithVelocity(Point4D.from(FINAL_POSE), 0.3);

    return drone.getTrajectory();
  }
}
