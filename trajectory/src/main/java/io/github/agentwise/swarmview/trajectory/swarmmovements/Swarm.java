package io.github.agentwise.swarmview.trajectory.swarmmovements;

import java.util.LinkedHashMap;
import java.util.Map;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
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

	public void script() {
		final double duration = 1.0;
		final double distance = 1.5;
		
		for (int i=0; i<3; i++) {
			drones.values().forEach(drone -> drone.moveUp(distance, duration));
			drones.values().forEach(drone -> drone.moveDown(distance, duration));
		}
		
		for (int i=0; i<4; i++) {
			//move square
			drones.values().forEach(drone -> drone.moveRight(distance, duration));
			drones.values().forEach(drone -> drone.moveForward(distance, duration));
			drones.values().forEach(drone -> drone.moveLeft(distance, duration));
			drones.values().forEach(drone -> drone.moveBackward(distance, duration));
		}
		
		Point4D center = Point4D.create(5, 4.5, 1.5, 0);
		final double durationAway = 1;
		final double distanceAway = 1.0;

		//reduce square size
		drones.values().forEach(drone -> drone.moveAway(center, -distanceAway, durationAway));
		
		for (int i=0; i<4; i++) {
			//grow square
			drones.values().forEach(drone -> drone.moveAway(center, distanceAway+1.0, durationAway+1.0));
			//grow square
			drones.values().forEach(drone -> drone.moveAway(center, -distanceAway-1.0, durationAway+1.0));
		}
		
		//circling
//		drones.values().forEach(drone -> drone.moveCircle(center, true, 10));
//		drones.values().forEach(drone -> drone.moveCircle(center, false, 10));
		
	}

}