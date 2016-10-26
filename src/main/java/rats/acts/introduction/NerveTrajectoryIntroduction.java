/**
 * 
 */
package rats.acts.introduction;

import java.util.ArrayList;

import applications.trajectory.LineTrajectory;
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
				{ 	initialPosition.x(),	initialPosition.y(), 		initialPosition.z(),  	0 },	// start position, with start time
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
                {       4,      5,              1.5,    0.57 },
                {       3.5,	5,	 	        1.5,    0.21 },
                {       4.5,	5,          	1.5,    0.41 },
                {       2,      2,              4,      2.0 },
                {       2,      2,              4,      1.5 },
                {       2,      2,              2.5,    1 },
                {       2,      2,              4,      0.62 },
                {       2,      2,              4,      0.5 },
                {       2,      2,              2.5,    0.8 },
				{ 	finalPosition.x(),	finalPosition.y(), 		finalPosition.z(),  	1 },
		};
		
		double startTime = path[0][3];
		Pose endPosition = Pose.create (path[0][0], path[0][1], path[0][2], 0);
		Pose startPosition;
		double duration = 0;
		
		LineTrajectory line;
		
		for(double[] lineInfo : path) {
			startTime += duration;
			duration = lineInfo[3];
			startPosition = endPosition;
			endPosition = Pose.create (lineInfo[0], lineInfo[1], lineInfo[2], 0);
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
		return line.getDesiredPosition (timeInSeconds);
	}

}
