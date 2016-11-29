/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.ShortSwingTrajectory4D;
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
      {0, 2, 6, 1, 2, 0.1}, // x1 y1 x2 y2 height frequency
      {6, 1, 2, 1, 2, 0.1},
      {2, 1, 6, 3, 2, 0.1},
      {6, 3, 1, 3, 2, 0.1},
      {1, 3, 4.5, 1.8, 2, 0.1},
      {4.5, 1.8, 0.5, 0.5, 2, 0.1},
      {0.5, 0.5, 6, 0, 2, 0.1},
      {6, 0, 0, 3.55, 2, 0.1},
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
      final double height = lineInfo[4];
      final double frequency = lineInfo[5];

      if (!currentPosition.equals(startPoint)) {
        trajectoryBuilder.addTrajectory(
            StraightLineTrajectory4D.createWithCustomVelocity(currentPosition, startPoint, 2));
      }

      trajectoryBuilder
          .addTrajectory(
              ShortSwingTrajectory4D.create(
                  endPoint, Point3D.project(startPoint), height, frequency))
          .withDuration(0.5 / frequency);

      currentPosition = endPoint;

      //      Point3D circleCenterPoint = Point3D.create(lineInfo[0], lineInfo[1], lineInfo[2]);
      //      double frequency = lineInfo[3];
      //      double radius = lineInfo[4];
      //      double revolutions = lineInfo[5];
      //      double duration = (1 / frequency) * revolutions;
      //      double direction = lineInfo[6];
      //
      //      Trajectory4d circleTraj =
      //          Trajectories.swingTrajectoryBuilder()
      //              .setRadius(radius)
      //              .setFrequency(frequency)
      //              .setOrigin(Point4D.from(circleCenterPoint, 0))
      //              .setXzPlaneAngle(direction)
      //              .build();
      //      Point4D startCircleTraj =
      //          Point4D.create(
      //              circleTraj.getDesiredPositionX(0),
      //              circleTraj.getDesiredPositionY(0),
      //              circleTraj.getDesiredPositionZ(0),
      //              circleTraj.getDesiredAngleZ(0));
      //      StraightLineTrajectory4D line =
      //          StraightLineTrajectory4D.createWithCustomVelocity(currentPosition, startCircleTraj, 1);
      //
      //      trajectoryBuilder.addTrajectory(line);
      //      trajectoryBuilder.addTrajectory(circleTraj).withDuration(duration);
      //      currentPosition = startCircleTraj;
    }

    // Go to final position
		if (!currentPosition.equals(finalPosition)) {
			StraightLineTrajectory4D gotoFinalPosition = StraightLineTrajectory4D.createWithPercentageVelocity(

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
