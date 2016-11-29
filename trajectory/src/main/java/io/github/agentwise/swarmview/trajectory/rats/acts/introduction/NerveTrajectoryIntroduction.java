/**
 *
 */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * @author tom
 *
 */
public class NerveTrajectoryIntroduction implements FiniteTrajectory4d {

	private double duration;
	private FiniteTrajectory4d trajectory;

	public NerveTrajectoryIntroduction (Pose initialPosition, Pose finalPosition, double start) throws Exception {
		final double yaw = -Math.PI/2;
		final double heigh1 = 1.8;
		final double low1 = 0.8;
		final double x1 = 6;
		final double y1 = 3;
		final double speed1 = 0.5;

		final double heigh2 = 3.2;
		final double low2 = 1.5;
		final double x2 = 3.5;
		final double y2 = 3.55;
		final double speed2 = 0.5;

		final double heigh3 = 3.5;
		final double low3 = 1.0;
		final double x3 = 2;
		final double y3 = 0;
		final double speed3 = 0.5;
		double[][] path = {
				{ 	initialPosition.x(),	initialPosition.y(), 		initialPosition.z(),  	yaw, 1.0 },	// start position, start time is ignored
				{    x1 - 0.1 +  -0.06 , y1 +  0.0 ,  heigh1  +  0.04 , yaw +  0.07 * 5, speed1 +  0.12 } ,
				{    x1 - 0.2 +  0.21 , y1 +  0.02 ,  low1  +  0.07 , yaw +  0.09 * 5, speed1 +  0.12 } ,
				{    x1 - 0.3 +  -0.08 , y1 +  0.02 ,  heigh1  +  0.07 , yaw +  -0.17 * 5, speed1 +  -0.09 } ,
				{    x1 - 0.4 +  0.21 , y1 +  0.01 ,  low1  +  0.0 , yaw +  -0.08 * 5, speed1 +  0.18 } ,
				{    x1 - 0.5 +  0.02 , y1 +  -0.07 ,  heigh1  +  -0.11 , yaw +  0.11 * 5, speed1 +  0.0 } ,
				{    x1 - 0.6 +  0.14 , y1 +  -0.1 ,  low1  +  -0.03 , yaw +  -0.02 * 5, speed1 +  0.02 } ,
				{    x1 - 0.7 +  -0.05 , y1 +  -0.12 ,  heigh1  +  0.17 , yaw +  0.15 * 5, speed1 +  -0.16 } ,
				{    x1 - 0.8 +  -0.09 , y1 +  -0.07 ,  low1  +  0.08 , yaw +  -0.15 * 5, speed1 +  -0.14 } ,
				{    x1 - 0.9 +  0.22 , y1 +  -0.06 ,  heigh1  +  -0.06 , yaw +  0.04 * 5, speed1 +  0.18 } ,
				{    x1 - 1.0 +  0.03 , y1 +  0.19 ,  low1  +  0.19 , yaw +  -0.04 * 5, speed1 +  0.19 } ,
				{    x1 - 1.1 +  -0.26 , y1 +  0.0 ,  heigh1  +  0.01 , yaw +  0.03 * 5, speed1 +  0.08 } ,
				{    x1 - 1.2 +  -0.09 , y1 +  -0.19 ,  low1  +  -0.11 , yaw +  -0.17 * 5, speed1 +  -0.05 } ,
				{    x1 - 1.3 +  0.28 , y1 +  0.03 ,  heigh1  +  0.15 , yaw +  -0.12 * 5, speed1 +  0.09 } ,
				{    x1 - 1.4 +  -0.01 , y1 +  0.15 ,  low1  +  -0.14 , yaw +  0.05 * 5, speed1 +  0.02 } ,
				{    x1 - 1.5 +  -0.02 , y1 +  -0.12 ,  heigh1  +  -0.12 , yaw +  -0.06 * 5, speed1 +  0.01 } ,
				{    x1 - 1.6 +  -0.26 , y1 +  -0.11 ,  low1  +  -0.06 , yaw +  0.08 * 5, speed1 +  0.06 } ,
				{    x1 - 1.7 +  0.09 , y1 +  -0.12 ,  heigh1  +  -0.14 , yaw +  0.0 * 5, speed1 +  0.09 } ,
				{    x1 - 1.8 +  0.1 , y1 +  0.01 ,  low1  +  -0.01 , yaw +  0.19 * 5, speed1 +  -0.04 } ,
				{    x1 - 1.9 +  0.12 , y1 +  -0.13 ,  heigh1  +  0.15 , yaw +  -0.04 * 5, speed1 +  0.12 } ,
				{    x1 - 2.0 +  0.03 , y1 +  -0.11 ,  low1  +  0.18 , yaw +  0.09 * 5, speed1 +  -0.11 } ,
				{    x1 - 2.1 +  -0.28 , y1 +  0.0 ,  heigh1  +  0.07 , yaw +  -0.05 * 5, speed1 +  -0.12 } ,
				{    x1 - 2.2 +  0.29 , y1 +  -0.02 ,  low1  +  0.06 , yaw +  -0.02 * 5, speed1 +  0.01 } ,
				{    x1 - 2.3 +  -0.28 , y1 +  0.13 ,  heigh1  +  0.09 , yaw +  0.15 * 5, speed1 +  -0.18 } ,
				{    x1 - 2.4 +  -0.19 , y1 +  -0.12 ,  low1  +  0.04 , yaw +  0.06 * 5, speed1 +  0.06 } ,
				{    x1 - 2.5 +  0.04 , y1 +  -0.2 ,  heigh1  +  0.02 , yaw +  -0.02 * 5, speed1 +  0.04 } ,
//				{    x1 - 2.5 +  0.25 , y1 +  0.02 ,  low1  +  -0.19 , yaw +  -0.04 * 5, speed1 +  -0.17 } ,
//				{    x1 - 2.5 +  -0.28 , y1 +  0.11 ,  heigh1  +  -0.03 , yaw +  0.08 * 5, speed1 +  0.1 } ,
//				{    x1 - 2.5 +  0.16 , y1 +  0.09 ,  low1  +  0.08 , yaw +  0.02 * 5, speed1 +  -0.04 } ,
//				{    x1 - 2.5 +  0.17 , y1 +  -0.16 ,  heigh1  +  -0.05 , yaw +  -0.12 * 5, speed1 +  0.16 } ,
//				{    x1 - 2.5 +  -0.28 , y1 +  -0.19 ,  low1  +  -0.11 , yaw +  0.06 * 5, speed1 +  -0.03 } ,
//				{    x1 - 2.5 +  -0.03 , y1 +  0.11 ,  heigh1  +  0.15 , yaw +  0.1 * 5, speed1 +  0.18 } ,
//				{    x1 - 2.5 +  -0.21 , y1 + 0.55 +  0.09 ,  low1  +  -0.16 , yaw +  -0.19 * 5, speed1 +  -0.08 } ,

				////////////////// PHASE 2 /////////////

				{    x2 - 0.0 +  -0.06 , y2 - 0 +  0.0 ,  heigh2  +  0.04 , yaw +  0.07 * 5, speed2 +  0.12 } ,
				{    x2 - 0.1 +  0.21 , y2 - 0.15 +  0.02 ,  low2  +  0.07 , yaw +  0.09 * 5, speed2 +  0.12 } ,
				{    x2 - 0.2 +  -0.08 , y2 - 0.30 +  0.02 ,  heigh2  +  0.07 , yaw +  -0.17 * 5, speed2 +  -0.09 } ,
				{    x2 - 0.3 +  0.21 , y2 - 0.45 +  0.01 ,  low2  +  0.0 , yaw +  -0.08 * 5, speed2 +  0.18 } ,
				{    x2 - 0.4 +  0.02 , y2 - 0.60 +  -0.07 ,  heigh2  +  -0.11 , yaw +  0.11 * 5, speed2 +  0.0 } ,
				{    x2 - 0.5 +  0.14 , y2 - 0.75 +  -0.1 ,  low2  +  -0.03 , yaw +  -0.02 * 5, speed2 +  0.02 } ,
				{    x2 - 0.6 +  -0.05 , y2 - 0.90 +  -0.12 ,  heigh2  +  0.17 , yaw +  0.15 * 5, speed2 +  -0.16 } ,
				{    x2 - 0.7 +  -0.09 , y2 - 1.05 +  -0.07 ,  low2  +  0.08 , yaw +  -0.15 * 5, speed2 +  -0.14 } ,
				{    x2 - 0.8 +  0.22 , y2 - 1.20 +  -0.06 ,  heigh2  +  -0.06 , yaw +  0.04 * 5, speed2 +  0.18 } ,
				{    x2 - 0.9 +  0.03 , y2 - 1.35 +  0.19 ,  low2  +  0.19 , yaw +  -0.04 * 5, speed2 +  0.19 } ,
				{    x2 - 1.0 +  -0.26 , y2 - 1.50 +  0.0 ,  heigh2  +  0.01 , yaw +  0.03 * 5, speed2 +  0.08 } ,
				{    x2 - 1.1 +  -0.09 , y2 - 1.65 +  -0.19 ,  low2  +  -0.11 , yaw +  -0.17 * 5, speed2 +  -0.05 } ,
				{    x2 - 1.2 +  0.28 , y2 - 1.80 +  0.03 ,  heigh2  +  0.15 , yaw +  -0.12 * 5, speed2 +  0.09 } ,
				{    x2 - 1.3 +  -0.01 , y2 - 1.95 +  0.15 ,  low2  +  -0.14 , yaw +  0.05 * 5, speed2 +  0.02 } ,
				{    x2 - 1.4 +  -0.02 , y2 - 2.10 +  -0.12 ,  heigh2  +  -0.12 , yaw +  -0.06 * 5, speed2 +  0.01 } ,
				{    x2 - 1.5 +  -0.26 , y2 - 2.25 +  -0.11 ,  low2  +  -0.06 , yaw +  0.08 * 5, speed2 +  0.06 } ,
				{    x2 - 1.6 +  0.09 , y2 - 2.40 +  -0.12 ,  heigh2  +  -0.14 , yaw +  0.0 * 5, speed2 +  0.09 } ,
				{    x2 - 1.5 +  0.1 , y2 - 2.55 +  0.01 ,  low2  +  -0.01 , yaw +  0.19 * 5, speed2 +  -0.04 } ,
				{    x2 - 1.5 +  0.12 , y2 - 2.70 +  -0.13 ,  heigh2  +  0.15 , yaw +  -0.04 * 5, speed2 +  0.12 } ,
				{    x2 - 1.5 +  0.03 , y2 - 2.85 +  -0.11 ,  low2  +  0.18 , yaw +  0.09 * 5, speed2 +  -0.11 } ,
				{    x2 - 1.5 +  -0.28 , y2 - 3.00 +  0.0 ,  heigh2  +  0.07 , yaw +  -0.05 * 5, speed2 +  -0.12 } ,
				{    x2 - 1.5 +  0.29 , y2 - 3.15 +  -0.02 ,  low2  +  0.06 , yaw +  -0.02 * 5, speed2 +  0.01 } ,
				{    x2 - 1.5 +  -0.28 , y2 - 3.30 +  0.13 ,  heigh2  +  0.09 , yaw +  0.15 * 5, speed2 +  -0.18 } ,
				{    x2 - 1.5 +  -0.19 , y2 - 3.45 +  -0.12 ,  low2  +  0.04 , yaw +  0.06 * 5, speed2 +  0.06 } ,
				{    x2 - 2.4 +  0.04 , y2 - 3.55 +  -0.2 ,  heigh2  +  0.02 , yaw +  -0.02 * 5, speed2 +  0.04 } ,
//				{    x2 - 2.5 +  0.25 , y2 - 3.75 +  0.02 ,  low2  +  -0.19 , yaw +  -0.04 * 5, speed2 +  -0.17 } ,
//				{    x2 - 2.6 +  -0.28 , y2 - 3.90 +  0.11 ,  heigh2  +  -0.03 , yaw +  0.08 * 5, speed2 +  0.1 } ,
//				{    x2 - 2.7 +  0.16 , y2 - 4.05 +  0.09 ,  low2  +  0.08 , yaw +  0.02 * 5, speed2 +  -0.04 } ,
//				{    x2 - 2.8 +  0.17 , y2 - 4.20 +  -0.16 ,  heigh2  +  -0.05 , yaw +  -0.12 * 5, speed2 +  0.16 } ,
//				{    x2 - 2.9 +  -0.28 , y2 - 4.35 +  -0.19 ,  low2  +  -0.11 , yaw +  0.06 * 5, speed2 +  -0.03 } ,
//				{    x2 - 3.0 +  -0.03 , y2 - 4.50 +  0.11 ,  heigh2  +  0.15 , yaw +  0.1 * 5, speed2 +  0.18 } ,
//				{    x2 - 1.5 +  -0.21 , y2 - 3.55 +  0.09 ,  low2  +  -0.16 , yaw +  -0.19 * 5, speed2 +  -0.08 } ,


				/////////////////// PHASE 3 //////////


				{    x3 +  -0.06 , y3 +  0.0 ,  heigh3  +  0.04 , yaw +  0.07 * 5, speed3 +  0.12 } ,
				{    x3 +  0.21 , y3 +  0.02 ,  low3  +  0.07 , yaw +  0.09 * 5, speed3 +  0.12 } ,
				{    x3 +  -0.08 , y3 +  0.02 ,  heigh3  +  0.07 , yaw +  -0.17 * 5, speed3 +  -0.09 } ,
				{    x3 +  0.21 , y3 +  0.01 ,  low3  +  0.0 , yaw +  -0.08 * 5, speed3 +  0.18 } ,
				{    x3 +  0.02 , y3 +  -0.07 ,  heigh3  +  -0.11 , yaw +  0.11 * 5, speed3 +  0.0 } ,
				{    x3 +  0.14 , y3 +  -0.1 ,  low3  +  -0.03 , yaw +  -0.02 * 5, speed3 +  0.02 } ,
				{    x3 +  -0.05 , y3 +  -0.12 ,  heigh3  +  0.17 , yaw +  0.15 * 5, speed3 +  -0.16 } ,
				{    x3 +  -0.09 , y3 +  -0.07 ,  low3  +  0.08 , yaw +  -0.15 * 5, speed3 +  -0.14 } ,
				{    x3 +  0.22 , y3 +  -0.06 ,  heigh3  +  -0.06 , yaw +  0.04 * 5, speed3 +  0.18 } ,
				{    x3 +  0.03 , y3 +  0.19 ,  low3  +  0.19 , yaw +  -0.04 * 5, speed3 +  0.19 } ,
				{    x3 +  -0.26 , y3 +  0.0 ,  heigh3  +  0.01 , yaw +  0.03 * 5, speed3 +  0.08 } ,
				{    x3 +  -0.09 , y3 +  -0.19 ,  low3  +  -0.11 , yaw +  -0.17 * 5, speed3 +  -0.05 } ,
				{    x3 +  0.28 , y3 +  0.03 ,  heigh3  +  0.15 , yaw +  -0.12 * 5, speed3 +  0.09 } ,
				{    x3 +  -0.01 , y3 +  0.15 ,  low3  +  -0.14 , yaw +  0.05 * 5, speed3 +  0.02 } ,
				{    x3 +  -0.02 , y3 +  -0.12 ,  heigh3  +  -0.12 , yaw +  -0.06 * 5, speed3 +  0.01 } ,
				{    x3 +  -0.26 , y3 +  -0.11 ,  low3  +  -0.06 , yaw +  0.08 * 5, speed3 +  0.06 } ,
				{    x3 +  0.09 , y3 +  -0.12 ,  heigh3  +  -0.14 , yaw +  0.0 * 5, speed3 +  0.09 } ,
				{    x3 +  0.1 , y3 +  0.01 ,  low3  +  -0.01 , yaw +  0.19 * 5, speed3 +  -0.04 } ,
				{    x3 +  0.12 , y3 +  -0.13 ,  heigh3  +  0.15 , yaw +  -0.04 * 5, speed3 +  0.12 } ,
				{    x3 +  0.03 , y3 +  -0.11 ,  low3  +  0.18 , yaw +  0.09 * 5, speed3 +  -0.11 } ,
				{    x3 +  -0.28 , y3 +  0.0 ,  heigh3  +  0.07 , yaw +  -0.05 * 5, speed3 +  -0.12 } ,
				{    x3 +  0.29 , y3 +  -0.02 ,  low3  +  0.06 , yaw +  -0.02 * 5, speed3 +  0.01 } ,
				{    x3 +  -0.28 , y3 +  0.13 ,  heigh3  +  0.09 , yaw +  0.15 * 5, speed3 +  -0.18 } ,
//				{    x3 +  -0.19 , y3 +  -0.12 ,  low3  +  0.04 , yaw +  0.06 * 5, speed3 +  0.06 } ,
//				{    x3 +  0.04 , y3 +  -0.2 ,  heigh3  +  0.02 , yaw +  -0.02 * 5, speed3 +  0.04 } ,
//				{    x3 +  0.25 , y3 +  0.02 ,  low3  +  -0.19 , yaw +  -0.04 * 5, speed3 +  -0.17 } ,
//				{    x3 +  -0.28 , y3 +  0.11 ,  heigh3  +  -0.03 , yaw +  0.08 * 5, speed3 +  0.1 } ,
//				{    x3 +  0.16 , y3 +  0.09 ,  low3  +  0.08 , yaw +  0.02 * 5, speed3 +  -0.04 } ,
//				{    x3 +  0.17 , y3 +  -0.16 ,  heigh3  +  -0.05 , yaw +  -0.12 * 5, speed3 +  0.16 } ,
//				{    x3 +  -0.28 , y3 +  -0.19 ,  low3  +  -0.11 , yaw +  0.06 * 5, speed3 +  -0.03 } ,
//				{    x3 +  -0.03 , y3 +  0.11 ,  heigh3  +  0.15 , yaw +  0.1 * 5, speed3 +  0.18 } ,
//				{    x3 +  -0.21 , y3 +  0.09 ,  low3  +  -0.16 , yaw +  -0.19 * 5, speed3 +  -0.08 } ,



				{ 	finalPosition.x(),	finalPosition.y(), 		finalPosition.z(),  	yaw, 0.5 },
		};

		double startTime = start;
		Point4D endPosition = Point4D.create (path[0][0], path[0][1], path[0][2], yaw);
		Point4D startPosition;
//		double duration = start;

		TrajectoryComposite.Builder trajectoryBuilder = TrajectoryComposite.builder();

		if (start > 0) {
			Hover hoverBeforeBegin = new Hover (initialPosition, start);
			trajectoryBuilder.addTrajectory(hoverBeforeBegin);
		}

		StraightLineTrajectory4D line;

		for(int i = 1 ; i < path.length; i++) {
			double[] lineInfo = path[i];
			startTime += duration;
			System.out.println(lineInfo[0]);
			double spd = lineInfo[4];
			startPosition = endPosition;
			endPosition = Point4D.create (lineInfo[0], lineInfo[1], lineInfo[2], lineInfo[3]);
			line = StraightLineTrajectory4D.createWithPercentageVelocity(startPosition, endPosition, spd);
			double lineDuration = line.getTrajectoryDuration();
			startTime += lineDuration;
			trajectoryBuilder.addTrajectory(line);
			System.out.println("Trajectory added, duration " + duration);
		}

		this.duration = startTime;
		this.trajectory = trajectoryBuilder.build();

	}

	@Override
	public double getTrajectoryDuration() {
		return this.duration;
	}


	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		return this.trajectory.getDesiredPosition (timeInSeconds);
	}

}


