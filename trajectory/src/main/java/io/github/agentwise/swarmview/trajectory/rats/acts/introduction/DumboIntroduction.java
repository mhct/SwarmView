/**
 *
 */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.WiggleTrajectory;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.ZigZagTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * @author tom
 *
 */
public class DumboIntroduction {

  private FiniteTrajectory4d trajectory;

  public DumboIntroduction(Pose initialPose, Pose finalPose, double start) {

    double[][] path = {{2, 2, 3, 2, 1, 15, 5.0},

        {5, 1, 2, 3, 1, 7, 5.0},
        // wiggle position, number of wiggles, time to stay at edge, number of zigzags to get
				// there, distance of zigzags
        {1, 5, 1, 2, 1, 12, 5.0},
        {6, 2, 3.5, 2, 1, 12, 5.0},
        {4, 2.5, 1.5, 2, 1, 5, 5.0},
        // wiggle position, number of wiggles, time to stay at edge, number of zigzags to get
				// there, distance of zigzags
    };

    TrajectoryComposite.Builder trajectoryBuilder = TrajectoryComposite.builder();

    if (start > 0) {
      Hover hoverBeforeBegin = new Hover(initialPose, start);
      trajectoryBuilder.addTrajectory(hoverBeforeBegin);
    }

    Point4D currentPosition = Point4D.from(initialPose);
    Point4D finalPosition = Point4D.from(finalPose);

    // First hover a bit more
    // Hover initHover = new Hover (currentPosition, 2);
    // trajectoryBuilder.addTrajectory(initHover);

    for (int i = 0; i < path.length; i++) {
      double[] lineInfo = path[i];

      // Go to wiggle position
      Point4D newWigglePosition = Point4D.create(lineInfo[0], lineInfo[1], lineInfo[2], -StrictMath.PI / 2);
      // StraightLineTrajectory4D gotoWigglePosition = StraightLineTrajectory4D
			// .createWithPercentageVelocity(currentPosition, newWigglePosition, 0.7);
      ZigZagTrajectory4D gotoWigglePosition = new ZigZagTrajectory4D(currentPosition,
          newWigglePosition, (int) lineInfo[5], lineInfo[6], 5.0);
      trajectoryBuilder.addTrajectory(gotoWigglePosition);
      currentPosition = newWigglePosition;

      // Now hover before wiggle
      Hover hoverBeforeWiggle = new Hover(currentPosition, 0.1);
      trajectoryBuilder.addTrajectory(hoverBeforeWiggle);

      // Now WIGGLE !!
      WiggleTrajectory wiggle = new WiggleTrajectory(currentPosition, (int) lineInfo[3],
          lineInfo[4]);
      trajectoryBuilder.addTrajectory(wiggle);

    }

    // Go to final position
    // StraightLineTrajectory4D gotoFinalPosition = StraightLineTrajectory4D
		// .createWithPercentageVelocity(currentPosition, finalPosition, 0.7);
    ZigZagTrajectory4D gotoFinalPosition = new ZigZagTrajectory4D(currentPosition, finalPosition, 4,
        0.5, 0.7);
    trajectoryBuilder.addTrajectory(gotoFinalPosition);

    this.trajectory = trajectoryBuilder.build();

  }

  public FiniteTrajectory4d getTrajectory() {
    return this.trajectory;
  }

  public static FiniteTrajectory4d createTrajectory(Pose initialPosition, Pose finalPosition,
      double startTime) {

    DumboIntroduction dumbo = new DumboIntroduction(initialPosition, finalPosition, startTime);
    return dumbo.getTrajectory();

  }

}
