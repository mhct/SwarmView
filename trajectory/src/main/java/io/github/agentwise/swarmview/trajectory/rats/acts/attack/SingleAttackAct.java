/**
 *
 */
package io.github.agentwise.swarmview.trajectory.rats.acts.attack;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.DronePositionConfiguration;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * @author tom
 *
 */
public class SingleAttackAct extends Act {

	private SingleAttackAct (ActConfiguration configuration) {
		super(configuration);
	}

	public static Act create (Point3D point3d, double standBackHeight, double attackHeight, double standBackRadius, double attackRadius) {

		AttackTrajectories trajectories = new AttackTrajectories(point3d, standBackHeight, attackHeight, standBackRadius, attackRadius);
		Map<DroneName, FiniteTrajectory4d> droneTrajectories = new LinkedHashMap<DroneName, FiniteTrajectory4d>();

		int numberOfDrones = DroneName.values().length;

		droneTrajectories.put(DroneName.Nerve, 	trajectories.getTrajectory (0, numberOfDrones, 0));
		droneTrajectories.put(DroneName.Romeo, 	trajectories.getTrajectory (1, numberOfDrones, 0.31));
		droneTrajectories.put(DroneName.Juliet, trajectories.getTrajectory (3, numberOfDrones, 0.31));
		droneTrajectories.put(DroneName.Fievel, trajectories.getTrajectory (2, numberOfDrones, 0.6));
		droneTrajectories.put(DroneName.Dumbo, 	trajectories.getTrajectory (4, numberOfDrones, 0.8));

		List<DronePositionConfiguration> positions = new ArrayList<>();
		for (DroneName drone: DroneName.values()) {
			Pose pose = droneTrajectories.get(drone).getDesiredPosition(0);
			positions.add(DronePositionConfiguration.create(drone,  pose, pose));
		}
		ActConfiguration configuration = ActConfiguration.create(positions);
		Act act = new SingleAttackAct (configuration);

		for (Map.Entry<DroneName, FiniteTrajectory4d> droneTraj : droneTrajectories.entrySet()) {
		    act.addTrajectory(droneTraj.getKey(), droneTraj.getValue());
		}

		act.lockAndBuild();

		return act;

	}

}
