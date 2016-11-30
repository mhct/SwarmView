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
		
		//
		// Perform the joint movements
		//
		Point4D center1 = Point4D.create(3.5, 2.6, 1.0, 0);

		Swarm swarm = Swarm.create(configuration.initialPositionConfiguration());
		swarm.setSwarmMovementsScript(new TamingSwarmScript(center1, act.finalPositions()));
		swarm.getDroneNames().forEach(drone -> act.addTrajectory(drone, swarm.get(drone)));
		
		return act;
	}
	
	/**
	 * Defines all the drone movements of the TamingAct.
	 * 
	 * @author Mario h.c.t.
	 *
	 */
	private static class TamingSwarmScript implements SwarmMovementsScript {

		private Point4D center1;
		private Map<DroneName, Pose> finalPositions;
		private static double YAW = -Math.PI/2;
		
		public TamingSwarmScript(Point4D center, Map<DroneName, Pose> finalPositions) {
			this.center1 = center;
			this.finalPositions = finalPositions;
		}
		
		@Override
		public void setSwarmMovementsScript(Map<DroneName, Particle> drones) {
			double radius = 1.8;
			Map<DroneName, Point4D> beginMovement = new LinkedHashMap<DroneName, Point4D>(5);
			
			beginMovement.put(Nerve, Point4D.pointAtAngle(center1, radius, 2*Math.PI/5));
			beginMovement.put(Romeo, Point4D.pointAtAngle(center1, radius, 4*Math.PI/5));
			beginMovement.put(Juliet, Point4D.pointAtAngle(center1, radius, 6*Math.PI/5));
			beginMovement.put(Dumbo, Point4D.pointAtAngle(center1, radius, 0));
			beginMovement.put(Fievel, Point4D.pointAtAngle(center1, radius, 8*Math.PI/5));
			
			final double duration = 1; 
			drones.forEach((drone, particle) -> particle.moveToPoint(beginMovement.get(drone), duration));
			
//			moveUpDown(drones);
//			moveAwayClose(drones);
//			moveBack(drones);
//			moveTwoCircles(drones);
//			drones.values().forEach(drone -> drone.moveHorizontalCircle(originCenter, true, 20, 0, 0.00001));
//			moveSpiral(drones);
			
			
			drones.get(Dumbo).moveBackward(1, 1);
			drones.get(Dumbo).moveHorizontalCircle(Point4D.pointAtAngle(center1, radius, 0), true, 10, 3, 0);
			drones.forEach((drone, particle) -> particle.moveToPoint(Point4D.from(finalPositions.get(drone)), duration));
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
		}
	}
}
