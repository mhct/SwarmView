/**
 * 
 */
package applications.trajectory;

import java.util.ArrayList;

import applications.trajectory.geom.point.Point3D;
import control.Trajectory4d;

/**
 * @author tom
 *
 */
public class NerveTrajectoryIntroduction implements Trajectory4d {

	private ArrayList<LineTrajectory> lineTrajectories = new ArrayList<LineTrajectory>();
	
	public NerveTrajectoryIntroduction () throws Exception {
		
		double[][] path = {
				{ 	7,	6, 		0.0,  	4 },	// start position, with start time
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
				{ 	2, 	2, 		4,  	2 },
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
	
	/* (non-Javadoc)
	 * @see control.Trajectory4d#getDesiredPositionX(double)
	 */
	@Override
	public double getDesiredPositionX(double timeInSeconds) {
		LineTrajectory line = this.getCurrentLineTrajectory(timeInSeconds);
		return line.getDesiredPositionX(timeInSeconds);
	}

	/* (non-Javadoc)
	 * @see control.Trajectory4d#getDesiredPositionY(double)
	 */
	@Override
	public double getDesiredPositionY(double timeInSeconds) {
		LineTrajectory line = this.getCurrentLineTrajectory(timeInSeconds);
		return line.getDesiredPositionY(timeInSeconds);
	}

	/* (non-Javadoc)
	 * @see control.Trajectory4d#getDesiredPositionZ(double)
	 */
	@Override
	public double getDesiredPositionZ(double timeInSeconds) {
		LineTrajectory line = this.getCurrentLineTrajectory(timeInSeconds);
		return line.getDesiredPositionZ(timeInSeconds);
	}

	/* (non-Javadoc)
	 * @see control.Trajectory4d#getDesiredAngleZ(double)
	 */
	@Override
	public double getDesiredAngleZ(double timeInSeconds) {
		// TODO Auto-generated method stub
		return 0;
	}

}
