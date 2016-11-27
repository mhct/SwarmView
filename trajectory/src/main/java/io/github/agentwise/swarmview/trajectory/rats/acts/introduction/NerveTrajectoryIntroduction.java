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
		double heigh = 3.5;
		double low = 1.5;
		double left = 3.5;
		double mid = 4;
		double right = 4.5;
		double depth = 3;
		double speed = 0.5;
		double yaw = -Math.PI/2;
		double[][] path = {
				{ 	initialPosition.x(),	initialPosition.y(), 		initialPosition.z(),  	yaw, 1.0 },	// start position, start time is ignored
				{    mid +  -0.06 , depth +  0.0 ,  heigh  +  0.04 * 3, yaw +  0.07 , speed +  0.12 } ,
				{    mid +  0.21 , depth +  0.02 ,  low  +  0.07 * 3, yaw +  0.09 , speed +  0.12 } ,
				{    mid +  -0.08 , depth +  0.02 ,  heigh  +  0.07 * 3, yaw +  -0.17 , speed +  -0.09 } ,
				{    mid +  0.21 , depth +  0.01 ,  low  +  0.0 * 3, yaw +  -0.08 , speed +  0.18 } ,
				{    mid +  0.02 , depth +  -0.07 ,  heigh  +  -0.11 * 3, yaw +  0.11 , speed +  0.0 } ,
				{    mid +  0.14 , depth +  -0.1 ,  low  +  -0.03 * 3, yaw +  -0.02 , speed +  0.02 } ,
				{    mid +  -0.05 , depth +  -0.12 ,  heigh  +  0.17 * 3, yaw +  0.15 , speed +  -0.16 } ,
				{    mid +  -0.09 , depth +  -0.07 ,  low  +  0.08 * 3, yaw +  -0.15 , speed +  -0.14 } ,
				{    mid +  0.22 , depth +  -0.06 ,  heigh  +  -0.06 * 3, yaw +  0.04 , speed +  0.18 } ,
				{    mid +  0.03 , depth +  0.19 ,  low  +  0.19 * 3, yaw +  -0.04 , speed +  0.19 } ,
				{    mid +  -0.26 , depth +  0.0 ,  heigh  +  0.01 * 3, yaw +  0.03 , speed +  0.08 } ,
				{    mid +  -0.09 , depth +  -0.19 ,  low  +  -0.11 * 3, yaw +  -0.17 , speed +  -0.05 } ,
				{    mid +  0.28 , depth +  0.03 ,  heigh  +  0.15 * 3, yaw +  -0.12 , speed +  0.09 } ,
				{    mid +  -0.01 , depth +  0.15 ,  low  +  -0.14 * 3, yaw +  0.05 , speed +  0.02 } ,
				{    mid +  -0.02 , depth +  -0.12 ,  heigh  +  -0.12 * 3, yaw +  -0.06 , speed +  0.01 } ,
				{    mid +  -0.26 , depth +  -0.11 ,  low  +  -0.06 * 3, yaw +  0.08 , speed +  0.06 } ,
				{    mid +  0.09 , depth +  -0.12 ,  heigh  +  -0.14 * 3, yaw +  0.0 , speed +  0.09 } ,
				{    mid +  0.1 , depth +  0.01 ,  low  +  -0.01 * 3, yaw +  0.19 , speed +  -0.04 } ,
				{    mid +  0.12 , depth +  -0.13 ,  heigh  +  0.15 * 3, yaw +  -0.04 , speed +  0.12 } ,
				{    mid +  0.03 , depth +  -0.11 ,  low  +  0.18 * 3, yaw +  0.09 , speed +  -0.11 } ,
				{    mid +  -0.28 , depth +  0.0 ,  heigh  +  0.07 * 3, yaw +  -0.05 , speed +  -0.12 } ,
				{    mid +  0.29 , depth +  -0.02 ,  low  +  0.06 * 3, yaw +  -0.02 , speed +  0.01 } ,
				{    mid +  -0.28 , depth +  0.13 ,  heigh  +  0.09 * 3, yaw +  0.15 , speed +  -0.18 } ,
				{    mid +  -0.19 , depth +  -0.12 ,  low  +  0.04 * 3, yaw +  0.06 , speed +  0.06 } ,
				{    mid +  0.04 , depth +  -0.2 ,  heigh  +  0.02 * 3, yaw +  -0.02 , speed +  0.04 } ,
				{    mid +  0.25 , depth +  0.02 ,  low  +  -0.19 * 3, yaw +  -0.04 , speed +  -0.17 } ,
				{    mid +  -0.28 , depth +  0.11 ,  heigh  +  -0.03 * 3, yaw +  0.08 , speed +  0.1 } ,
				{    mid +  0.16 , depth +  0.09 ,  low  +  0.08 * 3, yaw +  0.02 , speed +  -0.04 } ,
				{    mid +  0.17 , depth +  -0.16 ,  heigh  +  -0.05 * 3, yaw +  -0.12 , speed +  0.16 } ,
				{    mid +  -0.28 , depth +  -0.19 ,  low  +  -0.11 * 3, yaw +  0.06 , speed +  -0.03 } ,
				{    mid +  -0.03 , depth +  0.11 ,  heigh  +  0.15 * 3, yaw +  0.1 , speed +  0.18 } ,
				{    mid +  -0.21 , depth +  0.09 ,  low  +  -0.16 * 3, yaw +  -0.19 , speed +  -0.08 } ,
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


