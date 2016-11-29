package io.github.agentwise.swarmview.trajectory.rats.acts.taming;

import static io.github.agentwise.swarmview.trajectory.control.DroneName.Dumbo;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Romeo;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.rats.acts.interact.InterAct;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;
import io.github.agentwise.swarmview.trajectory.swarmmovements.SwarmScript;

public class TamingAct extends Act {

	private TamingAct(ActConfiguration configuration) {
		super(configuration);
		
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new TamingAct(configuration);
		
		Point4D center = Point4D.create(3.5, 2.6, 1.5, 0);
		double radius = 2.0;
		//
		// Go to initial positions
		//
		Map<DroneName, Pose> beginMovement = new LinkedHashMap<DroneName, Pose>(5);
		beginMovement.put(Nerve, Point4D.pointAtAngle(center, radius, 2*Math.PI/5).toPose());
		beginMovement.put(Romeo, Point4D.pointAtAngle(center, radius, 4*Math.PI/5).toPose());
		beginMovement.put(Juliet, Point4D.pointAtAngle(center, radius, 6*Math.PI/5).toPose());
		beginMovement.put(Dumbo, Point4D.pointAtAngle(center, radius, 0).toPose());
		beginMovement.put(Fievel, Point4D.pointAtAngle(center, radius, 8*Math.PI/5).toPose());
		
		List<DroneName> beginMovementOrder = Arrays.asList(Nerve, Romeo, Juliet, Fievel, Dumbo);
		ActConfiguration beginActConfiguration = ActConfiguration.createFromInitialFinalPositions(act.initialPositions(), beginMovement);
		Act goToPosition = InterAct.createWithOrderedSequentialMovement(beginActConfiguration, 1.0, beginMovementOrder);
		goToPosition.lockAndBuild();
		
		//
		// Perform the joint movements
		//
		Swarm swarm = Swarm.create(beginActConfiguration.finalPositionConfiguration());
		swarm.setScript(new TamingSwarmScript(center));

		//
		// Move to final positions
		//
		Map<DroneName, Pose> moveToFinalPositionMovement = new LinkedHashMap<DroneName, Pose>(5);
		moveToFinalPositionMovement.put(Nerve, swarm.getFinalPose(Nerve));
		moveToFinalPositionMovement.put(Romeo, swarm.getFinalPose(Romeo));
		moveToFinalPositionMovement.put(Fievel, swarm.getFinalPose(Fievel));
		moveToFinalPositionMovement.put(Dumbo, swarm.getFinalPose(Dumbo));
		moveToFinalPositionMovement.put(Juliet, swarm.getFinalPose(Juliet));
		ActConfiguration goUpActConfiguration = ActConfiguration.createFromInitialFinalPositions(moveToFinalPositionMovement, act.finalPositions());
		Act goToFinalPosition = InterAct.createWithSequentialMovement(goUpActConfiguration, 1.0);
		
		try {
			for (DroneName drone: DroneName.values()) {
				act.addTrajectory(drone, TrajectoryComposite.builder()
						.addTrajectory(goToPosition.getTrajectory(drone))
						.addTrajectory(swarm.get(drone))
						.addTrajectory(goToFinalPosition.getTrajectory(drone)).build());
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	
	/**
	 * Defines all the drone movements of the TamingAct.
	 * 
	 * @author Mario h.c.t.
	 *
	 */
	private static class TamingSwarmScript implements SwarmScript {

		private Point4D originCenter;

		public TamingSwarmScript(Point4D center) {
			this.originCenter = center;
		}
		
		@Override
		public void script(Map<DroneName, Particle> drones) {

			final double duration = 1.0;
			final double distanceHeight = 1.8;
			final double distanceHeightDumbo = 0.5;
			final double distanceHeightSecondRow = 1.0;
			final double distanceHeightThirdRow = 2.0;
			
			final double distance = 2.0;

			for (int i=0; i<3; i++) {
				drones.get(Dumbo).moveUp(distanceHeightDumbo, duration);
				drones.get(Fievel).moveUp(distanceHeightSecondRow, duration);
				drones.get(Nerve).moveUp(distanceHeightSecondRow, duration);
				drones.get(Juliet).moveUp(distanceHeightThirdRow, duration);
				drones.get(Romeo).moveUp(distanceHeightThirdRow, duration);
				
				drones.get(Dumbo).moveUp(-distanceHeightDumbo, duration);
				drones.get(Fievel).moveUp(-distanceHeightSecondRow, duration);
				drones.get(Nerve).moveUp(-distanceHeightSecondRow, duration);
				drones.get(Juliet).moveUp(-distanceHeightThirdRow, duration);
				drones.get(Romeo).moveUp(-distanceHeightThirdRow, duration);
			}

			
			final double distanceAway = 1.0;
			final double durationAway = 2.0;
//
//			
			for (int i=0; i<3; i++) {
				drones.values().forEach(drone -> drone.moveAway(originCenter, -distanceAway, durationAway));
				drones.values().forEach(drone -> drone.moveAway(originCenter, distanceAway, durationAway));
			}
			
			//circling
			drones.values().forEach(drone -> drone.moveCircle(originCenter, true, 10));
			drones.values().forEach(drone -> drone.moveCircle(originCenter, false, 10));
			drones.get(Dumbo).moveCircle(Point4D.plus(originCenter, Point4D.create(1, 0, 0, 0)), false, 2);
			drones.get(Fievel).moveCircle(originCenter, false, 4);
			drones.get(Nerve).moveCircle(originCenter, false, 6);
			drones.get(Juliet).moveCircle(originCenter, false, 8);
			drones.get(Romeo).moveCircle(originCenter, false, 10);
//			drones.values().forEach(drone -> drone.moveCircle(center, false, 10));
			
		}
		
	}
}
