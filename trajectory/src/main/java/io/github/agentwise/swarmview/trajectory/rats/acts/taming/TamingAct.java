package io.github.agentwise.swarmview.trajectory.rats.acts.taming;

import static io.github.agentwise.swarmview.trajectory.control.DroneName.Dumbo;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Romeo;

import java.util.LinkedHashMap;
import java.util.Map;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;
import io.github.agentwise.swarmview.trajectory.swarmmovements.SwarmMovementsScript;

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
		

		Swarm swarm = Swarm.create(configuration.initialPositionConfiguration());
		swarm.setSwarmMovementsScript(new TamingSwarmScript(act.finalPositions()));
		swarm.addDroneMovementsToAct(act);
		return act;
	}
	
	/**
	 * Defines all the drone movements of the TamingAct.
	 * 
	 * @author Mario h.c.t.
	 *
	 */
	private static class TamingSwarmScript implements SwarmMovementsScript {

		private Map<DroneName, Pose> finalPositions;
		private Point4D center1;
		private Point4D center2;
		private static double YAW = -Math.PI/2;
		
		public TamingSwarmScript(Map<DroneName, Pose> finalPositions) {
			//
			// Perform the joint movements
			//
			center1 = Point4D.create(3.5, 2.6, 1.0, YAW);
			center2 = Point4D.create(3.5, 2.6, 2.5, YAW);
			this.finalPositions = finalPositions;
		}
		
		@Override
		public void setSwarmMovementsScript(Map<DroneName, Particle> drones) {
			
			final double duration = 1; 
			final double durationInwards = 3; 
			final double outerCircleRadius = 1.6;
			final double innerCircleRadius = 0.8;
			final double durationSpiralingOut = 20;
			final double spiralRate = 0.05;
			final double durationSmallCircling = 10;
			final double distanceUp = 1.3;
			final double durationLargeCircling = 20;
			final double durationMoveToFinalPostions = 2;
			final double durationMoveToLand = 10;
			
			moveToCirclePosition(drones, center1, outerCircleRadius, duration);
			moveAwayClose(drones);
			moveToCirclePosition(drones, center1, innerCircleRadius, durationInwards);
			drones.values().forEach(particle -> particle.moveHorizontalCircle(center1, true, durationSmallCircling)); //small circle around Jeana
			drones.values().forEach(particle -> particle.moveHorizontalCircle(center1, true, durationSpiralingOut, 0, spiralRate)); //ouwards spiral
			drones.values().forEach(particle -> particle.moveUp(distanceUp, duration));
			drones.values().forEach(particle -> particle.moveHorizontalCircle(center1, true, durationLargeCircling)); //big circle high in the air

			moveBackAndForward(drones);
			moveToCirclePosition(drones, center2, innerCircleRadius, durationMoveToFinalPostions);
			drones.values().forEach(particle -> particle.moveHorizontalCircle(center2, false, durationSmallCircling)); //small circle around Jeana
			drones.values().forEach(particle -> particle.moveDown(center2.getZ()-0.5, durationMoveToLand));
			
			
			
			
//			drones.forEach((drone, particle) -> particle.moveToPoint(Point4D.from(finalPositions.get(drone)), durationMoveToFinalPostions)); // move to final positions
//			drones.forEach((drone, particle) -> particle.moveToPoint(Point4D.from(finalPositions.get(drone)), durationMoveToFinalPostions)); // move to final positions
			
		}
		
		private void moveToCirclePosition(Map<DroneName, Particle> drones, Point4D circleCenter, double circleRadius, double duration) {
			Map<DroneName, Point4D> positions = new LinkedHashMap<DroneName, Point4D>(drones.size());
			
			positions.put(Dumbo, Point4D.pointAtAngle(circleCenter, circleRadius, 0));
			positions.put(Nerve, Point4D.pointAtAngle(circleCenter, circleRadius, 2*Math.PI/5));
			positions.put(Romeo, Point4D.pointAtAngle(circleCenter, circleRadius, 4*Math.PI/5));
			positions.put(Juliet, Point4D.pointAtAngle(circleCenter, circleRadius, 6*Math.PI/5));
			positions.put(Fievel, Point4D.pointAtAngle(circleCenter, circleRadius, 8*Math.PI/5));
			
			
			drones.forEach((drone, particle) -> particle.moveToPoint(positions.get(drone), duration));
		}
		
		private void moveBackAndForward(Map<DroneName, Particle> drones) {
			final double duration = 2;
			final double height = 1.5;
			final double distance = 4.5;
			final double durationForward = 3;
			
			
			drones.get(Dumbo).moveToPoint(Point4D.create(3.5, 0, height, YAW), duration);
			drones.get(Fievel).moveToPoint(Point4D.create(1.5, 0, height, YAW), duration);
			drones.get(Romeo).moveToPoint(Point4D.create(4.5, 0, height, YAW), duration);
			drones.get(Juliet).moveToPoint(Point4D.create(2.5, 0, height, YAW), duration);
			drones.get(Nerve).moveToPoint(Point4D.create(6.0, 0, height, YAW), duration);
			
			drones.forEach((drone, particle) -> {
				if (drone == Dumbo) {
					particle.moveForwardGoingHighInBetween(distance, 1.25, 1.5, durationForward);
				} else {
					particle.moveForward(distance, durationForward);
				}
			});
			
		}
		
		
		private void moveSpiral(Map<DroneName, Particle> drones) {
			double durationUp = 1;
			double durationCircling = 40;
			double spiralRate = -0.05;
			double distanceAway = 1.0;
			
			drones.get(Fievel).moveAway(center1, distanceAway, durationUp);
			drones.get(Nerve).moveAway(center1, distanceAway, durationUp);
			drones.get(Dumbo).moveAway(center1, distanceAway, durationUp);
			drones.get(Juliet).moveAway(center1, distanceAway, durationUp);
			drones.get(Romeo).moveAway(center1, distanceAway, durationUp);
			
			drones.get(Fievel).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Nerve).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Dumbo).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Juliet).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Romeo).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
		}
		private void moveTwoCircles(Map<DroneName, Particle> drones) {
			double durationUp = 1;
			double durationCircling = 15;
			
			drones.get(Fievel).moveUp(1.5, durationUp);
			drones.get(Nerve).moveUp(1.5, durationUp);
			drones.get(Dumbo).moveAway(center1, -1.0, durationUp);
			drones.get(Juliet).moveAway(center1, -1.0, durationUp);
			drones.get(Romeo).moveAway(center1, -1.0, durationUp);
			
			drones.get(Fievel).moveHorizontalCircle(Point4D.create(center1.getX(), center1.getY(), center1.getZ() + 1.5, center1.getAngle()), true, durationCircling);
			drones.get(Nerve).moveHorizontalCircle(Point4D.create(center1.getX(), center1.getY(), center1.getZ() + 1.5, center1.getAngle()), true, durationCircling);
			drones.get(Dumbo).moveHorizontalCircle(center1, false, durationCircling);
			drones.get(Juliet).moveHorizontalCircle(center1, false, durationCircling);
			drones.get(Romeo).moveHorizontalCircle(center1, false, durationCircling);
			
			drones.get(Dumbo).moveAway(center1, 1.0, durationUp);
			drones.get(Juliet).moveAway(center1, 1.0, durationUp);
			drones.get(Romeo).moveAway(center1, 1.0, durationUp);
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
			
			double distanceAwayDumboFinal = 0.5;
			double distanceAwayFievelFinal = 2.0;
			double distanceAwayNerveFinal = 2.0;
			double distanceAwayJulietFinal = 1.5;
			double distanceAwayRomeoFinal = 1.5;
			
			for (int i=1; i<5; i++) {
				double multiplier = 1;
				if (i%2 == 1) {
					multiplier = 1;
				} else {
					multiplier = -1;
				}
				drones.get(Dumbo).moveAway(center1, multiplier * distanceAwayDumbo, duration);
				drones.get(Fievel).moveAway(center1, multiplier * distanceAwayFievel, duration);
				drones.get(Nerve).moveAway(center1, multiplier * distanceAwayNerve, duration);
				drones.get(Juliet).moveAway(center1, multiplier * distanceAwayJuliet, duration);
				drones.get(Romeo).moveAway(center1, multiplier * distanceAwayRomeo, duration);
			}
			drones.get(Dumbo).moveAway(center1, distanceAwayDumboFinal, duration);
			drones.get(Fievel).moveAway(center1, distanceAwayFievelFinal, duration);
			drones.get(Nerve).moveAway(center1, distanceAwayNerveFinal, duration);
			drones.get(Juliet).moveAway(center1, distanceAwayJulietFinal, duration);
			drones.get(Romeo).moveAway(center1, distanceAwayRomeoFinal, duration);
		}
	}
}
