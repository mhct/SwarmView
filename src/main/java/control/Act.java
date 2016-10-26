package control;

import java.util.HashMap;
import java.util.Map;

import control.dto.Pose;

/**
 * This class represents an act of a show. 
 * An act has the choreographies of several drones, specifying the initial and final position of each drone, 
 * involved in the dance show.
 * 
 * If a drone is not involved in a particular act, its location still has to be added to the act, to avoid
 * possible collisions.
 * 
 * An act always assumes its time to start at ZERO (0.0)
 * 
 * @author Mario h.c.t.
 *
 */
public class Act {
	private Map<DroneName, FiniteTrajectory4d> trajectories;
	private Map<DroneName, DronePositionConfiguration> dronePositions;
	private double duration;
	
	public Act(ActConfiguration configuration) {
		this.duration = configuration.duration();
		
		trajectories = new HashMap<>();
		dronePositions = new HashMap<>();
		for (DronePositionConfiguration droneConf: configuration.dronePositionConfiguration()) {
			dronePositions.put(droneConf.name(), droneConf);
		}
	}

	/**
	 * Returns the trajectory of a drone, for this act
	 * 
	 * @param droneName the name of a drone participating in the act
	 * @return trajectory of the drone in the act
	 */
	public FiniteTrajectory4d getTrajectory(DroneName drone) {
		return trajectories.get(drone);
	}

	/**
	 * Adds a trajectory of a particular drone to this act.
	 * Each drone participating in this act MUST add its trajectory in the act, using this method.
	 * Only ONE trajectory per drone.
	 * 
	 * @param droneName
	 * @param trajectory
	 */
	public void addTrajectory(DroneName droneName, FiniteTrajectory4d trajectory) {
		trajectories.put(droneName, trajectory);
	}

	/**
	 * Returns the specified initial position of a drone
	 * 
	 * @param droneName the name of a drone participating in the act
	 * @return initial specified position of the drone in this act
	 */
	public Pose initialPosition(DroneName droneName) {
		return dronePositions.get(droneName).initialPosition();
	}

	/**
	 * Returns the specified final position of a drone
	 * 
	 * @param droneName the name of a drone participating in the act
	 * @return final specified position of the drone in this act
	 */
	public Pose finalPosition(DroneName droneName) {
		return dronePositions.get(droneName).finalPosition();
	}
	
	public double getDuration() {
		return duration;
	}

}
