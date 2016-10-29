/**
 * 
 */
package rats.acts.introduction;

import applications.trajectory.Hover;
import applications.trajectory.StraightLineTrajectory4D;
import applications.trajectory.Trajectories;
import applications.trajectory.Trajectory4d;
import applications.trajectory.composites.TrajectoryComposite;
import applications.trajectory.composites.TrajectoryComposite.Builder;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.FiniteTrajectory4d;
import control.dto.Pose;

/**
 * @author tom
 *
 */
public class FievelIntroduction {
	
	private FiniteTrajectory4d trajectory;
	
	public FievelIntroduction (Pose initialPose, Pose finalPose, double start) {
		
		double[][] path = {
					{ 	4, 4, 2,		 0.4,  0.45,	1, 0 },	// mid point pendulum, frequency, radius, revolutions, direction
					{ 	4.1, 4, 2,		 0.4,  0.45,	1, Math.PI },	// mid point pendulum, frequency, radius, revolutions, direction
					{ 	4, 3.5, 2.5,	 0.3,  1,		2, Math.PI*3/4 },
					// { 	4, 2, 3,	 0.18, 1.2,		1, Math.PI },
					{ 	5, 5, 2,		 0.4,  0.5,		1, Math.PI*1.25 },
					{ 	5.1, 5, 2,		 0.4,  0.5,		1, 2*Math.PI-Math.PI*1.25 },
			};
		
		Point4D currentPosition = Point4D.from(initialPose);
		Point4D finalPosition 	= Point4D.from(finalPose);

		Builder trajectoryBuilder = TrajectoryComposite.builder();
		
		if (start > 0) {
			Hover hoverBeforeBegin = new Hover (initialPose, start);
			trajectoryBuilder.addTrajectory(hoverBeforeBegin);
		}

		for(int i = 0 ; i < path.length; i++) {
			double[] lineInfo = path[i];

			Point3D circleCenterPoint = Point3D.create( lineInfo[0], lineInfo[1], lineInfo[2]);
			double frequency = lineInfo[3];
			double radius = lineInfo[4];
			double revolutions = lineInfo[5];
		    double duration = (1 / frequency) * revolutions;
		    double direction = lineInfo[6];

			Trajectory4d circleTraj = Trajectories.swingTrajectoryBuilder().setRadius(radius)
                    .setFrequency(frequency).setOrigin(Point4D.from(circleCenterPoint, 0))
                    .setXzPlaneAngle(direction).build();
			Point4D startCircleTraj = Point4D.create (	circleTraj.getDesiredPositionX(0),
														circleTraj.getDesiredPositionY(0),
														circleTraj.getDesiredPositionZ(0),
														circleTraj.getDesiredAngleZ(0));
			StraightLineTrajectory4D line = StraightLineTrajectory4D.createWithCustomVelocity(currentPosition, startCircleTraj, 1);

			trajectoryBuilder.addTrajectory(line);
			trajectoryBuilder.addTrajectory(circleTraj).withDuration(duration);
			currentPosition = startCircleTraj;
		}

		// Go to final position
		StraightLineTrajectory4D gotoFinalPosition = StraightLineTrajectory4D.createWithPercentageVelocity(currentPosition, finalPosition, 0.8);
		trajectoryBuilder.addTrajectory(gotoFinalPosition);

		this.trajectory = trajectoryBuilder.build();

	}
	
    public FiniteTrajectory4d getTrajectory() {
    	return this.trajectory;
    }

	public static FiniteTrajectory4d createTrajectory(Pose initialPosition, Pose finalPosition, double start) {
		
		FievelIntroduction dumbo = new FievelIntroduction(initialPosition, finalPosition, start);
		return dumbo.getTrajectory();
		
	}

}
