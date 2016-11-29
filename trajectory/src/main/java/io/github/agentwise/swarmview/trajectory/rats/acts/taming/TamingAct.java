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
	
	private static Point4D center2 = Point4D.create(3.5, 2.0, 1.0, 0);
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new TamingAct(configuration);
		
		Point4D center1 = Point4D.create(3.5, 2.6, 1.0, 0);
		double radius = 1.8;
		//
		// Go to initial positions
		//
		Map<DroneName, Pose> beginMovement = new LinkedHashMap<DroneName, Pose>(5);
		beginMovement.put(Nerve, Point4D.pointAtAngle(center1, radius, 2*Math.PI/5).toPose());
		beginMovement.put(Romeo, Point4D.pointAtAngle(center1, radius, 4*Math.PI/5).toPose());
		beginMovement.put(Juliet, Point4D.pointAtAngle(center1, radius, 6*Math.PI/5).toPose());
		beginMovement.put(Dumbo, Point4D.pointAtAngle(center1, radius, 0).toPose());
		beginMovement.put(Fievel, Point4D.pointAtAngle(center1, radius, 8*Math.PI/5).toPose());
		
		List<DroneName> beginMovementOrder = Arrays.asList(Nerve, Romeo, Juliet, Fievel, Dumbo);
		ActConfiguration beginActConfiguration = ActConfiguration.createFromInitialFinalPositions(act.initialPositions(), beginMovement);
		Act goToPosition = InterAct.createWithOrderedSequentialMovement(beginActConfiguration, 1.0, beginMovementOrder);
		goToPosition.lockAndBuild();
		
		//
		// Perform the joint movements
		//
		Swarm swarm = Swarm.create(beginActConfiguration.finalPositionConfiguration());
		swarm.setScript(new TamingSwarmScript(center1));

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
		private static double YAW = -Math.PI/2;
		
		public TamingSwarmScript(Point4D center) {
			this.originCenter = center;
		}
		
		@Override
		public void script(Map<DroneName, Particle> drones) {
//			moveUpDown(drones);
//			moveAwayClose(drones);
//			moveBack(drones);
//			moveTwoCircles(drones);
//			drones.values().forEach(drone -> drone.moveCircle(originCenter, true, 20, 0, 0.00001));
			moveSpiral(drones);
		}
		
		private void moveBack(Map<DroneName, Particle> drones) {
			double duration = 2;
			
			drones.get(Dumbo).moveToPoint(Point4D.create(3.5, 0, 1, YAW), duration);
			drones.get(Fievel).moveToPoint(Point4D.create(1.5, 0, 1, YAW), duration);
			drones.get(Romeo).moveToPoint(Point4D.create(4.5, 0, 1, YAW), duration);
			drones.get(Juliet).moveToPoint(Point4D.create(2.5, 0, 1, YAW), duration);
			drones.get(Nerve).moveToPoint(Point4D.create(6.0, 0, 1, YAW), duration);
		}
		
		private void moveSpiral(Map<DroneName, Particle> drones) {
			double durationUp = 1;
			double durationCircling = 18;
			double spiralRate = 0.1;
			
			drones.get(Fievel).moveAway(originCenter, -1.0, durationUp);
			drones.get(Nerve).moveAway(originCenter, -1.0, durationUp);
			drones.get(Dumbo).moveAway(originCenter, -1.0, durationUp);
			drones.get(Juliet).moveAway(originCenter, -1.0, durationUp);
			drones.get(Romeo).moveAway(originCenter, -1.0, durationUp);
			
			drones.get(Fievel).moveCircle(originCenter, false, durationCircling, 0, spiralRate);
			drones.get(Nerve).moveCircle(originCenter, false, durationCircling, 0, spiralRate);
			drones.get(Dumbo).moveCircle(originCenter, false, durationCircling, 0, spiralRate);
			drones.get(Juliet).moveCircle(originCenter, false, durationCircling, 0, spiralRate);
			drones.get(Romeo).moveCircle(originCenter, false, durationCircling, 0, spiralRate);
		}
		private void moveTwoCircles(Map<DroneName, Particle> drones) {
			double durationUp = 1;
			double durationCircling = 15;
			
			drones.get(Fievel).moveUp(1.5, durationUp);
			drones.get(Nerve).moveUp(1.5, durationUp);
			drones.get(Dumbo).moveAway(originCenter, -1.0, durationUp);
			drones.get(Juliet).moveAway(originCenter, -1.0, durationUp);
			drones.get(Romeo).moveAway(originCenter, -1.0, durationUp);
			
			drones.get(Fievel).moveCircle(Point4D.create(originCenter.getX(), originCenter.getY(), originCenter.getZ() + 1.5, originCenter.getAngle()), true, durationCircling);
			drones.get(Nerve).moveCircle(Point4D.create(originCenter.getX(), originCenter.getY(), originCenter.getZ() + 1.5, originCenter.getAngle()), true, durationCircling);
			drones.get(Dumbo).moveCircle(originCenter, false, durationCircling);
			drones.get(Juliet).moveCircle(originCenter, false, durationCircling);
			drones.get(Romeo).moveCircle(originCenter, false, durationCircling);
			
			drones.get(Dumbo).moveAway(originCenter, 1.0, durationUp);
			drones.get(Juliet).moveAway(originCenter, 1.0, durationUp);
			drones.get(Romeo).moveAway(originCenter, 1.0, durationUp);
			drones.get(Fievel).moveDown(1.5, durationUp);
			drones.get(Nerve).moveDown(1.5, durationUp);
		}
		
		private void moveUpDown(Map<DroneName, Particle> drones) {
			final double duration = 1.0;
			final double distanceHeightDumbo = 0.5;
			final double distanceHeightSecondRow = 1.0;
			final double distanceHeightThirdRow = 2.0;
			

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
		}
		
		private void moveAwayClose(Map<DroneName, Particle> drones) {
			final double duration = 2.5;
			double distanceAwayDumbo = 0.5;
			double distanceAwayFievel = 2.0;
			double distanceAwayNerve = 2.0;
			double distanceAwayJuliet = 1.5;
			double distanceAwayRomeo = 1.5;
			
			for (int i=1; i<5; i++) {
				double multiplier = 1;
				if (i%2 == 1) {
					multiplier = 1;
				} else {
					multiplier = -1;
				}
				drones.get(Dumbo).moveAway(originCenter, multiplier * distanceAwayDumbo, duration);
				drones.get(Fievel).moveAway(originCenter, multiplier * distanceAwayFievel, duration);
				drones.get(Nerve).moveAway(originCenter, multiplier * distanceAwayNerve, duration);
				drones.get(Juliet).moveAway(originCenter, multiplier * distanceAwayJuliet, duration);
				drones.get(Romeo).moveAway(originCenter, multiplier * distanceAwayRomeo, duration);
			}
		}
	}
}
