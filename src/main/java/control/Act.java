package control;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import applications.trajectory.HoldPositionTrajectory4D;
import applications.trajectory.composites.TrajectoryComposite;
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
 * TODO add builder to finalize the creation of an Act and calculate its duration
 * @author Mario h.c.t.
 *
 */
public class Act {
	private Map<DroneName, FiniteTrajectory4d> trajectories;
	private Map<DroneName, DronePositionConfiguration> dronePositions;
	private double duration;
	private boolean objectLocked;
	private LinkedHashMap<DroneName, Pose> initialPositions;
	private LinkedHashMap<DroneName, Pose> finalPositions;
	
	public Act(ActConfiguration configuration) {
		trajectories = new HashMap<>();
		dronePositions = new HashMap<>();
		for (DronePositionConfiguration droneConf: configuration.dronePositionConfiguration()) {
			dronePositions.put(droneConf.name(), droneConf);
		}
		setInitialAndFinalPositions();
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
		if (!isLocked()) {
			trajectories.put(droneName, trajectory);
		} else {
			throw new RuntimeException("Act instance can not be modified after locked");
		}
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

	/**
	 * Returns the specified initial position of ALL drones participating in the act
	 *
	 * @return Map<DroneName,Pose> with all drones and respective initial positions in this act
	 */
	public Map<DroneName, Pose> initialPositions() {
		return initialPositions;
	}
	
	
	/**
	 * Returns the specified final position of ALL drones participating in the act
	 *
	 * @return Map<DroneName,Pose> with all drones and respective initial positions in this act
	 */
	public Map<DroneName, Pose> finalPositions() {
		return finalPositions;
	}
	
	public double getDuration() {
		return duration;
	}
	
	/**
	 * Locks the current instance of the Act, not allowing new modifications.
	 */
	public void lockAndBuild() {
		
		setDuration();
		addPositionHoldShortTrajectories();
		objectLocked = true;
	}

	/**
	 * 
	 */
	private void setInitialAndFinalPositions() {
		initialPositions = new LinkedHashMap<>(dronePositions.size());
		for (Map.Entry<DroneName, DronePositionConfiguration> e: dronePositions.entrySet()) {
			initialPositions.put(e.getKey(), e.getValue().initialPosition());
		}
		
		finalPositions = new LinkedHashMap<>(dronePositions.size());
		for (Map.Entry<DroneName, DronePositionConfiguration> e: dronePositions.entrySet()) {
			finalPositions.put(e.getKey(), e.getValue().finalPosition());
		}
	}
	
	private void setDuration() {
		double maxDuration = 0;
		for (Map.Entry<DroneName, FiniteTrajectory4d> e: trajectories.entrySet()) {
			if (e.getValue().getTrajectoryDuration() - maxDuration >= 0.001) {
				maxDuration = e.getValue().getTrajectoryDuration();
			}
		}
		duration = maxDuration;
	}
	
	private void addPositionHoldShortTrajectories() {
		for (Map.Entry<DroneName, FiniteTrajectory4d> e: trajectories.entrySet()) {
			if (e.getValue().getTrajectoryDuration() - getDuration() <= -0.001) {
				double timeDelta = Math.abs(e.getValue().getTrajectoryDuration() - getDuration());
				e.setValue(TrajectoryComposite.builder().
						addTrajectory(e.getValue()).
						addTrajectory(HoldPositionTrajectory4D.create(
										finalPosition(e.getKey()))
									  ).withDuration(timeDelta).build()
						);
			}
		}
	}
	
	private boolean isLocked() {
		return objectLocked;
	}

}
