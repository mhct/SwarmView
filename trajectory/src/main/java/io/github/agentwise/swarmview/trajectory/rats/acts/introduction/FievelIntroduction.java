/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/** @author tom */
public class FievelIntroduction {

  private FiniteTrajectory4d trajectory;

  private static final double YAW = -StrictMath.PI / 2;

  public FievelIntroduction(Pose initialPose, Pose finalPose, double start) {

    final double altitude = 3;

    double[][] path = {
      {0, 2, 6, 1, 2, 0.05}, // x1 y1 x2 y2 height frequency
      {6, 1, 2, 1, 2, 0.04},
      {2, 1, 6, 3, 2, 0.04},
      {6, 3, 1, 3, 2, 0.04},
      {1, 3, 4.5, 1.8, 2, 0.05},
      {4.5, 1.8, 0.5, 0.5, 2, 0.05},
      {0.5, 0.5, 6, 0, 2, 0.04},
      {6, 0, 0, 3.55, 2, 0.03},
    };

    Point4D currentPosition = Point4D.from(initialPose);
    Point4D finalPosition = Point4D.from(finalPose);

    Builder trajectoryBuilder = TrajectoryComposite.builder();

    if (start > 0) {
      Hover hoverBeforeBegin = new Hover(initialPose, start);
      trajectoryBuilder.addTrajectory(hoverBeforeBegin);
    }

    for (int i = 0; i < path.length; i++) {
      double[] lineInfo = path[i];
      final Point4D startPoint = Point4D.create(lineInfo[0], lineInfo[1], altitude, YAW);
      final Point4D endPoint = Point4D.create(lineInfo[2], lineInfo[3], altitude, YAW);

      final Point4D middlePoint =
          Point4D.create(
              startPoint.getX() + (endPoint.getX() - startPoint.getX()) / 2,
              startPoint.getY() + (endPoint.getY() - startPoint.getY()) / 2,
              1,
              YAW);

//      if (!startPoint.equals(startPoint)) {
//        trajectoryBuilder.addTrajectory(
//            StraightLineTrajectory4D.createWithCustomVelocity(startPoint, startPoint, 2));
//      }

      trajectoryBuilder
          .addTrajectory(
              StraightLineTrajectory4D.createWithCustomVelocity(
                  startPoint, middlePoint, 2));

      trajectoryBuilder
          .addTrajectory(
              StraightLineTrajectory4D.createWithCustomVelocity(
                  middlePoint, endPoint, 2));

      currentPosition = endPoint;
    }

    // Go to final position
    if (!currentPosition.equals(finalPosition)) {
      StraightLineTrajectory4D gotoFinalPosition =
          StraightLineTrajectory4D.createWithPercentageVelocity(
              currentPosition, finalPosition, 0.8);
      trajectoryBuilder.addTrajectory(gotoFinalPosition);
    }

    this.trajectory = trajectoryBuilder.build();
  }

  public FiniteTrajectory4d getTrajectory() {
    return this.trajectory;
  }

  public static FiniteTrajectory4d createTrajectory(
      Pose initialPosition, Pose finalPosition, double start) {

    FievelIntroduction dumbo = new FievelIntroduction(initialPosition, finalPosition, start);
    return dumbo.getTrajectory();
  }
}
