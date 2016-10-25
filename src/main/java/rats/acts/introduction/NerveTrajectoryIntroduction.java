/**
 * 
 */
package rats.acts.introduction;

import java.util.ArrayList;

import applications.trajectory.LineTrajectory;
import applications.trajectory.geom.point.Point3D;
import control.FiniteTrajectory4d;
import control.dto.Pose;

/**
 * @author tom
 *
 */
public class NerveTrajectoryIntroduction implements FiniteTrajectory4d {

	private ArrayList<LineTrajectory> lineTrajectories = new ArrayList<LineTrajectory>();
	private double duration;
	
	public NerveTrajectoryIntroduction (Pose initialPosition, Pose finalPosition) throws Exception {
		double[][] path = {
				{ 	initialPosition.x(),	initialPosition.y(), 		initialPosition.z(),  	4 },	// start position, with start time
				{ 	7,	6, 		0.5,  	1 },	// go into position
				{ 	7,	6, 		0.5,  	1 },
				{ 	2, 	4, 		1.5,  	2.5 },
				{ 	5, 	3, 		4,  	2 },
				{ 3.5, 	7, 		2,  	2 },
				{ 1.5, 	4.5, 	2.5,  	1.6 },
				{ 6.5, 	2, 		1.5,  	2.4 },
				{ 	5, 	6, 		1.5, 	2.4 },
				{ 	2, 	2, 		4,  	2.5 },
				{ 	2, 	2, 		4,  	1 },
				{ 	2, 	2, 		1,  	1.5 },
				{ 	2, 	2, 		4,  	1.5 },
				{ 	2, 	2, 		4,  	1 },
				{ 	2, 	2, 		2.5,  	0.8 },
				{ 	2, 	2, 		4,  	1 },
				{ 	finalPosition.x(),	finalPosition.y(), 		finalPosition.z(),  	2 },
		};
		
		double startTime = path[0][3];
		Point3D endPosition = Point3D.create (path[0][0], path[0][1], path[0][2]);
		Point3D startPosition;
		double duration = 0;
		
		LineTrajectory line;
		
		for(double[] lineInfo : path) {
			startTime += duration;
			duration = lineInfo[3];
			startPosition = endPosition;
			endPosition = Point3D.create (lineInfo[0], lineInfo[1], lineInfo[2]);
			line = new LineTrajectory(startPosition, endPosition, startTime, startTime+duration);
			this.lineTrajectories.add(line);
		}
		
		this.duration = startTime + duration;

	}
	
	
	private LineTrajectory getCurrentLineTrajectory(double timeInSeconds) {
		if (this.lineTrajectories.get(0).startsAfter(timeInSeconds))
			return this.lineTrajectories.get(0);
		
		for (LineTrajectory line : this.lineTrajectories) {
			if (line.isActive(timeInSeconds))
				return line;
		}
		return this.lineTrajectories.get(this.lineTrajectories.size()-1);
	}
	
	@Override
	public double getTrajectoryDuration() {
		return duration;
	}


	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		LineTrajectory line = this.getCurrentLineTrajectory(timeInSeconds);
		return Pose.create(line.getDesiredPositionX(timeInSeconds),
				line.getDesiredPositionY(timeInSeconds),
				line.getDesiredPositionZ(timeInSeconds),
				line.getDesiredAngleZ(timeInSeconds));
	}

}
