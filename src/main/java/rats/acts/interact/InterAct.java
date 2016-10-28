/**
 * 
 */
package rats.acts.interact;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import applications.trajectory.StraightLineTrajectory4D;
import applications.trajectory.Trajectories;
import applications.trajectory.composites.TrajectoryComposite;
import applications.trajectory.composites.TrajectoryComposite.Builder;
import applications.trajectory.geom.point.Point4D;
import control.Act;
import control.ActConfiguration;
import control.DroneName;
import control.DronePositionConfiguration;
import control.dto.Pose;

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
			
		Act act = new InterAct(configuration);
		
		int number = 0;
		for (DroneName drone : DroneName.values()) {

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

		List<DronePositionConfiguration> positions = new ArrayList<>();

		for (DroneName drone : DroneName.values()) {
			positions.add(DronePositionConfiguration.create(drone, initialPositions.get(drone),  finalPositions.get(drone)));
		}
		ActConfiguration configuration = ActConfiguration.create(positions); //1"

		return createWithSequentialMovement (configuration, timeBetween);
	}

	public static Act create (Act prevAct, Act nextAct) {
		
		return create(prevAct.finalPositions(), nextAct.initialPositions());

	}
	
	

}
