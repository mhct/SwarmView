/**
 * 
 */
package io.github.agentwise.swarmview.rats.acts.interact;

import java.util.List;
import java.util.Map;

import io.github.agentwise.swarmview.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.applications.trajectory.Trajectories;
import io.github.agentwise.swarmview.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.control.Act;
import io.github.agentwise.swarmview.control.ActConfiguration;
import io.github.agentwise.swarmview.control.DroneName;
import io.github.agentwise.swarmview.control.dto.Pose;

/**
 * @author tom
 *
 */
public class InterAct extends Act {
	
	private static final double TIME_BETWEEN_STARTS = 1.0;

	private InterAct (ActConfiguration configuration) {
		super(configuration);
	}

	public static Act create (ActConfiguration configuration) {
		return createWithSequentialMovement(configuration, 0.0);
	}

	public static Act createWithSequentialMovement (ActConfiguration configuration) {
		return createWithSequentialMovement (configuration, TIME_BETWEEN_STARTS);
	}
	
	public static Act createWithSequentialMovement (ActConfiguration configuration, double timeBetween) {
		return createWithOrderedSequentialMovement(configuration, timeBetween, null);
	}
	
	public static Act createWithOrderedSequentialMovement (ActConfiguration configuration, double timeBetween, List<DroneName> movementOrder) {
		DroneName[] droneMovementOrder = new DroneName[DroneName.values().length];
		
		// Initializes the drone movement order
		if (movementOrder != null && movementOrder.size() == DroneName.values().length) {
			droneMovementOrder = movementOrder.toArray(droneMovementOrder);
		} else {
			droneMovementOrder = DroneName.values();
		}
		
		Act act = new InterAct(configuration);
		
		int number = 0;
		for (DroneName drone : droneMovementOrder) {

			Builder trajectoryBuilder = TrajectoryComposite.builder();

			Pose initPos	= act.initialPosition(drone);
			Point4D initPoint4D = Point4D.create(initPos.x(), initPos.y(), initPos.z(), initPos.yaw());
			Pose finalPos 	= act.finalPosition(drone);
			Point4D finalPoint4D = Point4D.create(finalPos.x(), finalPos.y(), finalPos.z(), finalPos.yaw());
			
			try {
				double timeBeforeStart = 0.1 + number * timeBetween;
				trajectoryBuilder
					.addTrajectory(
							Trajectories.newHoldPositionTrajectory(initPoint4D))
					.withDuration(timeBeforeStart);

				trajectoryBuilder
					.addTrajectory(StraightLineTrajectory4D.createWithPercentageVelocity(
																			initPoint4D,
																			finalPoint4D,
																			0.6));
				act.addTrajectory(drone, trajectoryBuilder.build());
				
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			number++;
		}
		
		return act;
	}

	public static Act create (Map<DroneName,Pose> initialPositions, Map<DroneName,Pose> finalPositions) {
		return createWithSequentialMovement (initialPositions, finalPositions, 0.0);
	}

	public static Act createWithSequentialMovement (Map<DroneName,Pose> initialPositions, Map<DroneName,Pose> finalPositions) {
		return createWithSequentialMovement (initialPositions, finalPositions, TIME_BETWEEN_STARTS);
	}
	
	public static Act createWithSequentialMovement (Map<DroneName,Pose> initialPositions, Map<DroneName,Pose> finalPositions, double timeBetween) {
		ActConfiguration configuration = ActConfiguration.createFromInitialFinalPositions(initialPositions, finalPositions);
		return createWithSequentialMovement (configuration, timeBetween);
	}

	public static Act create (Act prevAct, Act nextAct) {
		
		return create(prevAct.finalPositions(), nextAct.initialPositions());

	}
	
	

}
