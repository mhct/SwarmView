/**
 * 
 */
package io.github.agentwise.rats.acts.attack;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import io.github.agentwise.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.control.Act;
import io.github.agentwise.control.ActConfiguration;
import io.github.agentwise.control.DroneName;
import io.github.agentwise.control.DronePositionConfiguration;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;
import io.github.agentwise.rats.acts.chaos.ChaosAct;

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
		droneTrajectories.put(DroneName.Romeo, 	trajectories.getTrajectory (1, numberOfDrones, 1));
		droneTrajectories.put(DroneName.Juliet, trajectories.getTrajectory (3, numberOfDrones, 1));
		droneTrajectories.put(DroneName.Fievel, trajectories.getTrajectory (2, numberOfDrones, 2));
		droneTrajectories.put(DroneName.Dumbo, 	trajectories.getTrajectory (4, numberOfDrones, 3));

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
