package control;

import java.util.HashMap;
import java.util.Map;

import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;

import applications.trajectory.geom.point.Point3D;

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
	Map<DroneName, FiniteTrajectory4d> trajectories;
	Multimap<DroneName, Point3D> dronePositions = ArrayListMultimap.create();
	
	public Act() {
		trajectories = new HashMap<DroneName, FiniteTrajectory4d>();
		
//		initializeTrajectories();
	}

	/**
	 * Returns the trajectory of a drone, for this act
	 * 
	 * @param drone
	 * @return
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

}
