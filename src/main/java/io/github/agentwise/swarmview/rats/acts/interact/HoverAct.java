/**
 * 
 */
package io.github.agentwise.swarmview.rats.acts.interact;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import io.github.agentwise.swarmview.applications.trajectory.Hover;
import io.github.agentwise.swarmview.control.Act;
import io.github.agentwise.swarmview.control.ActConfiguration;
import io.github.agentwise.swarmview.control.DroneName;
import io.github.agentwise.swarmview.control.DronePositionConfiguration;
import io.github.agentwise.swarmview.control.dto.Pose;

/**
 * @author tom
 *
 */
public class HoverAct extends Act {

	private HoverAct(ActConfiguration configuration) {
		super(configuration);
	}

	public static Act create (Map<DroneName,Pose> hoverPositions, double duration) {

		List<DronePositionConfiguration> positions = new ArrayList<>();

		for (DroneName drone : DroneName.values()) {
			positions.add(DronePositionConfiguration.create(drone, hoverPositions.get(drone),  hoverPositions.get(drone)));
		}
		ActConfiguration configuration = ActConfiguration.create(positions);

		return create (configuration, duration);
	}
	
	private static Act create (ActConfiguration configuration, double duration) {
		Act act = new HoverAct(configuration);
		
		for (DroneName drone : DroneName.values()) {
			act.addTrajectory(
					drone,
					new Hover (act.initialPosition(drone), duration));
		}
		return act;
		
	}

}
