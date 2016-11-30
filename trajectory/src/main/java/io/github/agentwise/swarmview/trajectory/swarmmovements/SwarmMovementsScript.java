package io.github.agentwise.swarmview.trajectory.swarmmovements;

import java.util.Map;

import io.github.agentwise.swarmview.trajectory.control.DroneName;

/**
 * Defines the interface used by the Swarm to get the concrete movements
 * for the swarm.
 * 
 * @author Mario h.c.t.
 *
 */
public interface SwarmMovementsScript {
	/**
	 * Defines the movements each drone in the swarm will do
	 * 
	 * @param drones the drones in the swarm
	 * 
	 */
	public void setSwarmMovementsScript(Map<DroneName, Particle> drones);
}
