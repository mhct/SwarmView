/**
 * 
 */
package io.github.agentwise.swarmview.rats.acts.introduction;

import io.github.agentwise.swarmview.applications.trajectory.Hover;
import io.github.agentwise.swarmview.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.applications.trajectory.WiggleTrajectory;
import io.github.agentwise.swarmview.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.control.dto.Pose;

/**
 * @author tom
 *
 */
public class DumboIntroduction {

	private FiniteTrajectory4d trajectory;
	
	public DumboIntroduction (Pose initialPose, Pose finalPose, double start) {
		
		double[][] path = {
					{ 	4, 4, 1.5,		3, 1 },	// wiggle position, number of wiggles, time to stay at edge
					{ 	6, 5, 1,		2, 1 },	// wiggle position, number of wiggles, time to stay at edge
					{ 	2, 5, 2,		2, 1 },	// wiggle position, number of wiggles, time to stay at edge
			};
		
		Builder trajectoryBuilder = TrajectoryComposite.builder();

		if (start > 0) {
			Hover hoverBeforeBegin = new Hover (initialPose, start);
			trajectoryBuilder.addTrajectory(hoverBeforeBegin);
		}

		Point4D currentPosition = Point4D.from(initialPose);
		Point4D finalPosition 	= Point4D.from(finalPose);

		// First hover a bit more
		Hover initHover = new Hover (currentPosition, 2);
		trajectoryBuilder.addTrajectory(initHover);
	
		for(int i = 0 ; i < path.length; i++) {
			double[] lineInfo = path[i];

			// Go to wiggle position
			Point4D newWigglePosition = Point4D.create(lineInfo[0], lineInfo[1], lineInfo[2], 0);
			StraightLineTrajectory4D gotoWigglePosition = StraightLineTrajectory4D.createWithPercentageVelocity(currentPosition, newWigglePosition, 0.7);
			trajectoryBuilder.addTrajectory(gotoWigglePosition);
			currentPosition = newWigglePosition;
			
			// Now hover before wiggle
			Hover hoverBeforeWiggle = new Hover (currentPosition, 0.5);
			trajectoryBuilder.addTrajectory(hoverBeforeWiggle);

			// Now WIGGLE !!
			WiggleTrajectory wiggle = new WiggleTrajectory(currentPosition, (int) lineInfo[3], lineInfo[4]);
			trajectoryBuilder.addTrajectory(wiggle);


		}

		// Go to final position
		StraightLineTrajectory4D gotoFinalPosition = StraightLineTrajectory4D.createWithPercentageVelocity(currentPosition, finalPosition, 0.7);
		trajectoryBuilder.addTrajectory(gotoFinalPosition);

		this.trajectory = trajectoryBuilder.build();

	}
	
    public FiniteTrajectory4d getTrajectory() {
    	return this.trajectory;
    }

	public static FiniteTrajectory4d createTrajectory(Pose initialPosition, Pose finalPosition, double startTime) {
		
		DumboIntroduction dumbo = new DumboIntroduction(initialPosition, finalPosition, startTime);
		return dumbo.getTrajectory();
		
	}

}
