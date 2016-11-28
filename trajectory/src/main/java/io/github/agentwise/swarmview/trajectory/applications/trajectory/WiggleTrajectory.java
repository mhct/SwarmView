package io.github.agentwise.swarmview.trajectory.applications.trajectory;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * Trajectory that performs a wiggle effect in place. Note that this trajectory does not keep to the
 * usual enterVelocity restraints.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public class WiggleTrajectory extends BasicTrajectory implements FiniteTrajectory4d {
  private static final double WIGGLE_DISTANCE = 1.0d;
  private static final double TIME_TO_REST_AT_ORIGIN = 0.001d;
  private final FiniteTrajectory4d target;

  public WiggleTrajectory(Point4D centerPoint, int wiggles, double timeToStayAtEdges) {
    TrajectoryComposite.Builder builder = TrajectoryComposite.builder();
    double orientation = centerPoint.getAngle();
    Point4D endRight =
        Point4D.create(
            centerPoint.getX() + StrictMath.cos(orientation) * WIGGLE_DISTANCE,
            centerPoint.getY() + StrictMath.sin(orientation) * WIGGLE_DISTANCE,
            centerPoint.getZ(),
            orientation);
    Point4D endLeft =
        Point4D.create(
            centerPoint.getX() - StrictMath.cos(orientation) * WIGGLE_DISTANCE,
            centerPoint.getY() - StrictMath.sin(orientation) * WIGGLE_DISTANCE,
            centerPoint.getZ(),
            orientation);

    builder
    	.addTrajectory(StraightLineTrajectory4D.createWithCustomVelocity(centerPoint, endLeft, 5.0));

    for (int i = 0; i < wiggles; i++) {
      builder
          	.addTrajectory(StraightLineTrajectory4D.createWithCustomVelocity(endLeft, endRight, 5.0));
      builder
      		.addTrajectory(StraightLineTrajectory4D.createWithCustomVelocity(endRight, endLeft, 5.0));
    }
    builder
    	.addTrajectory(StraightLineTrajectory4D.createWithCustomVelocity(endLeft, centerPoint, 5.0));

    target = builder.build();
  }

  @Override
  public double getTrajectoryDuration() {
    return target.getTrajectoryDuration();
  }

  @Override
  public Pose getDesiredPosition(double timeInSeconds) {
    return Pose.create(target.getDesiredPosition(timeInSeconds));
  }
}
