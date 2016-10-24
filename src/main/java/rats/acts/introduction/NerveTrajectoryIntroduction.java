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
	
	public NerveTrajectoryIntroduction () throws Exception {
		
		double[][] path = {
                {       7,      6,              1,      4       },      // start position, with start time
                {       3,      5,              1.5,    1.7 },
                {       4,      5,              2,      0.5 },
                {       2,      5,              2,      0.82 },
                {       3,      6,              1.5,    0.65 },
                {       2,      6,              1.5,    0.42 },
                {       3,      6,              1.5,    0.41 },
                {       2,      5,              2.5,    0.7 },
                {       4,      5,              1.5,    0.91 },
                {       3,      5,              2.5,    0.57 },
                {       4,      5,              1.5,    0.57 },
                {       3.5,	5,	 	        1.5,    0.21 },
                {       4.5,	5,          	1.5,    0.41 },
                {       2,      2,              4,      2.0 },
                {       2,      2,              4,      1.5 },
                {       2,      2,              2.5,    1 },
                {       2,      2,              4,      0.62 },
                {       2,      2,              4,      0.5 },
                {       2,      2,              2.5,    0.8 },
                {       2,      2,              4,      0.8 },
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
