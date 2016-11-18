package io.github.agentwise.swarmview.trajectory.operationaltests;

import java.util.Map;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;
import io.github.agentwise.swarmview.trajectory.swarmmovements.SwarmScript;

public class BasicCirclesAct extends Act {

	private BasicCirclesAct(ActConfiguration configuration) {
		super(configuration);
		
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new BasicCirclesAct(configuration);
		
		//
		// Perform the joint movements
		//
		Swarm swarm = Swarm.create(configuration.initialPositionConfiguration());
		swarm.setScript(new BasicCircleSwarmScript());

		for (DroneName drone: swarm.getDroneNames()) {
			act.addTrajectory(drone, swarm.get(drone));
		}
		
		return act;
	}
	
	/**
	 * Defines a swarm of drones flying in a circle
	 * 
	 * @author Mario h.c.t.
	 *
	 */
	private static class BasicCircleSwarmScript implements SwarmScript {

		@Override
		public void script(Map<DroneName, Particle> drones) {

			final double duration = 60.0;
			Point4D center = Point4D.create(4.0, 3.0, 1.0, OperationalTestsShow.YAW);

			//circling
			drones.values().forEach(drone -> drone.moveCircle(center, true, duration));
			drones.values().forEach(drone -> drone.moveCircle(center, false, duration));
			
		}
		
	}
}
