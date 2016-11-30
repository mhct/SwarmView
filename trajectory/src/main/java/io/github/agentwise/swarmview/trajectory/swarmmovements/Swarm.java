package io.github.agentwise.swarmview.trajectory.swarmmovements;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * This class represents a swarm of drones. It allows to send 
 * commands to multiple drones at the same time, expecting their movement 
 * is somehow coordinated.
 * 
 * @author Mario h.c.t.
 *
 */
public class Swarm {
	Map<DroneName, Particle> drones;
	
	private Swarm(Map<DroneName, Pose> initialConfiguration) {
		drones = new LinkedHashMap<>();
		for (Map.Entry<DroneName, Pose> entry: initialConfiguration.entrySet()) {
			drones.put(entry.getKey(), new Particle(entry.getValue()));
		}
	}
	
	public static Swarm create(Map<DroneName, Pose> initialConfiguration) {
		Swarm swarm = new Swarm(initialConfiguration);
		return swarm;
	}
	
	public Pose getFinalPose(DroneName drone) {
		return get(drone).getDesiredPosition(get(drone).getTrajectoryDuration()); //bad bad bad
	}

	public FiniteTrajectory4d get(DroneName drone) {
		return drones.get(drone).getTrajectory();
	}

	/**
	 * Defines what is the script to be followed by the Swarm.
	 * 
	 * @param script
	 */
	public void setSwarmMovementsScript(SwarmMovementsScript script) {
		script.setSwarmMovementsScript(drones);
	}

	public Set<DroneName> getDroneNames() {
		return drones.keySet();
	}

}