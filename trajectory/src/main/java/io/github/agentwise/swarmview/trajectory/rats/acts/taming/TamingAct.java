package io.github.agentwise.swarmview.trajectory.rats.acts.taming;

import static io.github.agentwise.swarmview.trajectory.control.DroneName.Dumbo;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Romeo;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.KKK;

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
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new TamingAct(configuration);
		

		//hack to remove ROMEO 
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
		private Point4D center11;
		private Point4D center2;
		private static double YAW = -Math.PI/2;
		
		public TamingSwarmScript(Map<DroneName, Pose> finalPositions) {
			//
			// Perform the joint movements
			//
			center1 = Point4D.create(3, 3, 1, YAW);
			center11 = Point4D.create(3, 3, 2, YAW);
			center2 = Point4D.create(3, 3, 2.5, YAW);
			this.finalPositions = finalPositions;
		}
		
		@Override
		public void setSwarmMovementsScript(Map<DroneName, Particle> drones) {
			final Point4D center3 = Point4D.create(3, 2.5, 2.5, YAW);
			
			final double duration = 1; 
			final double durationInwards = 3; 
			final double outerCircleRadius = 1.3;
			final double innerCircleRadius = 1.3;
			final double durationSmallCircling = 20;
			final double durationLargeCircling = 20;
			
			double durationMoveBackwards = 1.2;
			double distanceBackwards = 0.2;

			//waiting drones before going initial positions...
			drones.values().forEach(particle -> particle.hover(5));
			
			final double durationToPosition = 3; 
			moveToCirclePosition(drones, center1, outerCircleRadius, durationToPosition);
			moveAwayClose(drones);
			moveToCirclePosition(drones, center11, innerCircleRadius, durationInwards);
			drones.values().forEach(particle -> particle.moveHorizontalCircle(center3, true, durationSmallCircling)); //small circle around Jeana
			
//			final double durationSpiralingOut = 10;
//			final double spiralRate = 0.02;
//			drones.values().forEach(particle -> particle.moveHorizontalCircle(center1, true, durationSpiralingOut, 0, spiralRate)); //ouwards spiral

			// REMOVE?
//			drones.values().forEach(particle -> particle.moveAway(center1, -0.7, 0.5));
//			drones.values().forEach(particle -> particle.moveHorizontalCircle(center3, true, durationLargeCircling)); //big circle high in the air
			
			moveBackAndForward(drones);
			moveToDiagonalLine(drones, 3);
			moveToDiagonalLine(drones, 3);
//			
			double minX = 0.3;
			double maxX = 5.7;
			moveHorizontalLineZ(drones, 1.5, minX, maxX);
			moveHorizontalLineZ(drones, 1.5, minX, maxX);
			moveHorizontalLineZ(drones, 1.5, minX, maxX);
//			moveVerticalV(drones, 1.5, 1.2, 4.8);
//			
//			final Point4D center4 = Point4D.create(3, 2.0, 2.5, YAW);
//			drones.values().forEach(particle -> particle.moveHorizontalCircle(center4, true, durationLargeCircling)); //big circle high in the air
		}
		
		
		private void land(Map<DroneName, Particle> drones) {
			drones.get(Romeo).moveDown(3, 5);
			
			drones.get(Fievel).hover(5);
			drones.get(Fievel).moveDown(2, 5);
			
			drones.get(Dumbo).hover(10);
			drones.get(Dumbo).moveDown(1, 5);
			
			drones.get(Nerve).hover(15);
			drones.get(Nerve).moveDown(0.5, 5);
		}
		
		/**
		 * @param drones
		 * @param duration
		 */
		private void moveToDiagonalLine(Map<DroneName, Particle> drones, double duration) {
			double numSpacesBetweenDrones = drones.size() - 1; // hardcoding for 4 drones only
//			double numDrones = drones.size();
			double minY = 1.2;
			double maxY = 4.7;
			double minX = 0.3;
			double maxX = 5.7;
			double minZ = 1.2;
			double maxZ = 3.2;
			
			double distX = (maxX - minX)/numSpacesBetweenDrones;
			double distY = (maxY - minY)/numSpacesBetweenDrones;
			double distZ = (maxZ - minZ)/numSpacesBetweenDrones;
			
			//first movement
			Point4D a = Point4D.create(minX + 0 * distX, minY + 0 * distY, maxZ - 0 * distZ, YAW); 
			Point4D b = Point4D.create(minX + 1 * distX, minY + 1 * distY, maxZ - 1 * distZ, YAW); 
			Point4D c = Point4D.create(minX + 2 * distX, minY + 2 * distY, maxZ - 2 * distZ, YAW); 
			Point4D d = Point4D.create(minX + 3 * distX, minY + 3 * distY, maxZ - 3 * distZ, YAW); 
			Point4D e = Point4D.create(minX + 4 * distX, minY + 4 * distY, maxZ - 4 * distZ, YAW); 
			
			drones.get(Dumbo).moveToPoint(a, duration);
			drones.get(Nerve).moveToPoint(b, duration);
			drones.get(Romeo).moveToPoint(c, duration);
			drones.get(Juliet).moveToPoint(d, duration);
			drones.get(Fievel).moveToPoint(e, duration);

			drones.values().forEach(particle -> particle.hover(2));
			
			//second movement
			a = Point4D.create(minX + 0 * distX, minY + 4 * distY, maxZ - 4 * distZ, YAW); 
			b = Point4D.create(minX + 1 * distX, minY + 3 * distY, maxZ - 3 * distZ, YAW); 
			c = Point4D.create(minX + 2.001 * distX, minY + 2 * distY, maxZ - 2 * distZ, YAW); 
			d = Point4D.create(minX + 3 * distX, minY + 1 * distY, maxZ - 1 * distZ, YAW); 
			e = Point4D.create(minX + 4 * distX, minY + 0 * distY, maxZ - 0 * distZ, YAW); 
			
			drones.get(Dumbo).moveToPoint(a, duration);
			drones.get(Nerve).moveToPoint(b, duration);
			drones.get(Romeo).moveToPoint(c, duration);
			drones.get(Juliet).moveToPoint(d, duration);
			drones.get(Fievel).moveToPoint(e, duration);
			
			drones.values().forEach(particle -> particle.hover(2));
		}
		
		
		private void moveVerticalV(Map<DroneName, Particle> drones, double duration, double minX, double maxX) {
			double numSpaces = 4; // hardcoding for 5 drones only
			double numSpacesZ = 2;
//			double numSpaces = drones.size() - 1;
			double minY = 1;
			double maxY = 5;
			
			double minZ = 1.2;
			double maxZ = 3.3;
			
			double distX = (maxX - minX)/numSpaces;
			double distY = (maxY - minY)/numSpaces;
			double distZ = (maxZ - minZ)/numSpacesZ;
			
			//First
			Point4D a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 2 * distZ, YAW); 
			Point4D b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
			Point4D c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 0.01 * distZ, YAW); 
			Point4D d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
			Point4D e = Point4D.create(minX + 4 * distX, minY + 1.5 * distY, minZ + 2 * distZ, YAW); 
			
			//Second
//			a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 0.1 * distZ, YAW); 
//			b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
//			c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 2 * distZ, YAW); 
//			d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 3 * distZ, YAW); 
			
			drones.get(Fievel).moveToPoint(a, duration);
			drones.get(Romeo).moveToPoint(b, duration);
			drones.get(Dumbo).moveToPoint(c, duration);
			drones.get(Nerve).moveToPoint(d, duration);
			drones.get(Juliet).moveToPoint(e, duration);
			
			drones.values().forEach(particle -> particle.hover(2));

			//third
//			a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 3 * distZ, YAW); 
//			b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 2 * distZ, YAW); 
//			c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
//			d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 0.1 * distZ, YAW); 
			
//			drones.get(Fievel).moveToPoint(a, duration);
//			drones.get(Romeo).moveToPoint(b, duration);
//			drones.get(Dumbo).moveToPoint(c, duration);
//			drones.get(Nerve).moveToPoint(d, duration);
//			drones.values().forEach(particle -> particle.hover(2));
		}
		
		private void moveHorizontalLineZ(Map<DroneName, Particle> drones, double duration, double minX, double maxX) {
			double numSpacesBetweenDrones = drones.size() - 1;
			double minY = 1;
			double maxY = 5;
			
			double minZ = 1.5;
			double maxZ = 3;
			
			double distX = (maxX - minX)/numSpacesBetweenDrones;
			double distY = (maxY - minY)/numSpacesBetweenDrones;
			double distZ = (maxZ - minZ)/numSpacesBetweenDrones;
			
			//
			//Second
			//
			Point4D a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 0.1 * distZ, YAW); 
			Point4D b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
			Point4D c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 2 * distZ, YAW); 
			Point4D d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 3 * distZ, YAW); 
			Point4D e = Point4D.create(minX + 4 * distX, minY + 1.5 * distY, minZ + 4 * distZ, YAW); 
			
			drones.get(Dumbo).moveToPoint(a, duration);
			drones.get(Nerve).moveToPoint(b, duration);
			drones.get(Romeo).moveToPoint(c, duration);
			drones.get(Juliet).moveToPoint(d, duration);
			drones.get(Fievel).moveToPoint(e, duration);
			
			drones.values().forEach(particle -> particle.hover(2));

			//
			//third
			//
			a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 4 * distZ, YAW); 
			b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 3 * distZ, YAW); 
			c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 2.01 * distZ, YAW); 
			d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
			e = Point4D.create(minX + 4 * distX, minY + 1.5 * distY, minZ + 0.01 * distZ, YAW); 
			
			drones.get(Dumbo).moveToPoint(a, duration);
			drones.get(Nerve).moveToPoint(b, duration);
			drones.get(Romeo).moveToPoint(c, duration);
			drones.get(Juliet).moveToPoint(d, duration);
			drones.get(Fievel).moveToPoint(e, duration);
			
			drones.values().forEach(particle -> particle.hover(2));
		}
		private void moveToCirclePosition(Map<DroneName, Particle> drones, Point4D circleCenter, double circleRadius, double duration) {
			drones.get(Dumbo).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 0*Math.PI/5), duration);
			drones.get(Nerve).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 2*Math.PI/5), duration);
			drones.get(Romeo).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 4*Math.PI/5), duration);
			drones.get(Juliet).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 6*Math.PI/5), duration);
			drones.get(Fievel).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 8*Math.PI/5), duration);
		}
		
		private void moveBackAndForward(Map<DroneName, Particle> drones) {
			final int numSpaceBetweenDrones = drones.size() - 1; 
			final double duration = 6;
			final double height = 2.3;
			final double distanceForward = 3.5;
			final double durationForward = 3;
			final double minY = 0.5;
			final double minX = 0.3;
			final double maxX = 5.7;
			final double distX = (maxX - minX)/numSpaceBetweenDrones;
			
			DroneName a = Dumbo;
			DroneName b = Nerve;
			DroneName c = Romeo;
			DroneName d = Juliet;
			DroneName e = Fievel;
			
			drones.get(a).moveToPoint(Point4D.create(minX + 0 * distX, minY, height, YAW), duration);
			drones.get(b).moveToPoint(Point4D.create(minX + 1 * distX, minY, height, YAW), duration);
			drones.get(c).moveToPoint(Point4D.create(minX + 2 * distX, minY, height, YAW), duration);
			drones.get(d).moveToPoint(Point4D.create(minX + 3 * distX, minY, height, YAW), duration);
			drones.get(e).moveToPoint(Point4D.create(minX + 4 * distX, minY, height, YAW), duration);
			
			drones.values().forEach(drone -> drone.hover(3));
			
			drones.forEach((drone, particle) -> {
				if (drone == b || drone == d) {
					particle.moveForwardGoingHighInBetween(distanceForward, 1.25, 1.5, durationForward);
					particle.hover(5);

				} else {
					particle.moveForward(distanceForward, durationForward);
					particle.hover(5);
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
			drones.get(Romeo).moveAway(center1, distanceAway, durationUp);
//			drones.get(Romeo).moveAway(center1, distanceAway, durationUp);
			
			drones.get(Fievel).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Nerve).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Dumbo).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
			drones.get(Romeo).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
//			drones.get(Romeo).moveHorizontalCircle(center1, false, durationCircling, 0, spiralRate);
		}
		private void moveEllipse(Map<DroneName, Particle> drones) {
			double durationUp = 1;
			double durationCircling = 15;
			
			double minX = 0;
			double minY = 1;
			
			double a = 5.5;
			double b = 4;
			
			drones.get(Fievel).moveUp(1.5, durationUp);
			drones.get(Nerve).moveUp(1.5, durationUp);
			drones.get(Dumbo).moveAway(center1, -1.0, durationUp);
			drones.get(Romeo).moveAway(center1, -1.0, durationUp);
//			drones.get(Romeo).moveAway(center1, -1.0, durationUp);
			
			drones.get(Fievel).moveHorizontalCircle(Point4D.create(center1.getX(), center1.getY(), center1.getZ() + 1.5, center1.getAngle()), true, durationCircling);
			drones.get(Nerve).moveHorizontalCircle(Point4D.create(center1.getX(), center1.getY(), center1.getZ() + 1.5, center1.getAngle()), true, durationCircling);
			drones.get(Dumbo).moveHorizontalCircle(center1, false, durationCircling);
			drones.get(Romeo).moveHorizontalCircle(center1, false, durationCircling);
//			drones.get(Romeo).moveHorizontalCircle(center1, false, durationCircling);
			
			drones.get(Dumbo).moveAway(center1, 1.0, durationUp);
			drones.get(Romeo).moveAway(center1, 1.0, durationUp);
//			drones.get(Romeo).moveAway(center1, 1.0, durationUp);
			drones.get(Fievel).moveDown(1.5, durationUp);
			drones.get(Nerve).moveDown(1.5, durationUp);
		}
		
		private void moveUp(Map<DroneName, Particle> drones, double duration) {
			final double distanceHeightDumbo = 0.5;
			final double distanceHeightSecondRow = 1.0;
			final double distanceHeightThirdRow = 2.0;
			
			drones.get(Dumbo).moveUp(distanceHeightDumbo, duration);
			drones.get(Fievel).moveUp(distanceHeightSecondRow, duration);
			drones.get(Nerve).moveUp(distanceHeightSecondRow, duration);
			drones.get(Romeo).moveUp(distanceHeightThirdRow, duration);
			drones.get(Juliet).moveUp(distanceHeightThirdRow, duration);
		}
		
		//TODO move up as well
		private void moveAwayClose(Map<DroneName, Particle> drones) {
			final double duration = 2.5;
			double distanceAwayDumbo = 0.5;
			double distanceAwayFievel = 1.2;
			double distanceAwayNerve = 1.0;
			double distanceAwayJulietRomeo = 1.0;
			
			double distanceAwayDumboFinal = 0.5;
			double distanceAwayFievelFinal = 1.2;
			double distanceAwayNerveFinal = 1.0;
			double distanceAwayJulietRomeoFinal = 0.2;
			
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
				drones.get(Romeo).moveAway(center1, multiplier * distanceAwayJulietRomeo, duration);
				drones.get(Juliet).moveAway(center1, multiplier * distanceAwayJulietRomeo, duration);
				drones.values().forEach(particle -> particle.hover(0.5));
			}
			drones.get(Dumbo).moveAway(center1, distanceAwayDumboFinal, duration);
			drones.get(Fievel).moveAway(center1, distanceAwayFievelFinal, duration);
			drones.get(Nerve).moveAway(center1, distanceAwayNerveFinal, duration);
			drones.get(Romeo).moveAway(center1, distanceAwayJulietRomeoFinal, duration);
			drones.get(Juliet).moveAway(center1, distanceAwayJulietRomeoFinal, duration);
			drones.values().forEach(particle -> particle.hover(0.5));
		}
	}
}
