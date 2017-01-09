package io.github.agentwise.swarmview.trajectory.rats.acts.taming;

import static io.github.agentwise.swarmview.trajectory.control.DroneName.Dumbo;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Romeo;

import java.util.List;
import java.util.Map;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

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

		private Point4D center1;
		private Point4D center11;
		private static double YAW = -Math.PI/2;
		
		public TamingSwarmScript(Map<DroneName, Pose> finalPositions) {
			//
			// Perform the joint movements
			//
			center1 = Point4D.create(3, 3, 1, YAW);
			center11 = Point4D.create(3, 3, 2, YAW);
		}
		
		@Override
		public void setSwarmMovementsScript(Map<DroneName, Particle> drones) {
			final Point4D center3 = Point4D.create(3, 2.5, 2.5, YAW);
			
			final double durationInwards = 3; 
			final double outerCircleRadius = 1.3;
			final double innerCircleRadius = 1.4;
			final double durationSmallCircling = 24;

			//waiting drones before going initial positions...
			drones.values().forEach(particle -> particle.hover(5));
			
			final double durationToPosition = 3; 
			moveToCirclePosition(drones, center1, outerCircleRadius, durationToPosition);

			moveAwayClose(drones);
			moveToCirclePosition(drones, center11, innerCircleRadius, durationInwards);
			drones.values().forEach(particle -> particle.moveHorizontalCircle(center11, true, 0.04, durationSmallCircling)); //small circle around Jeana
			
			moveBackwardAndForward(drones);
			
			moveToDiagonalLine(drones, 4.5);
			moveToDiagonalLine(drones, 4.5);

			double minX = 0.3;
			double maxX = 5.7;
			moveHorizontalLineZ(drones, 3.0, minX, maxX);
			moveHorizontalLineZ(drones, 3.0, minX, maxX);
//			moveHorizontalLineZ(drones, 3.0, minX, maxX);

			final Point4D center = moveVerticalV(drones, 3.0, 1.4, 4.6);
			
			final double durationFinalCircle = 28.5;
			drones.get(Fievel).moveHorizontalCircle(center, true, 0.03, durationFinalCircle); //big circle high in the air
			drones.get(Juliet).moveHorizontalCircle(center, true, 0.03, durationFinalCircle); //big circle high in the air
			drones.get(Dumbo).moveHorizontalCircle(center, true, 0.03, durationFinalCircle); //big circle high in the air
			drones.get(Romeo).moveHorizontalCircle(center, true, 0.03, durationFinalCircle); //big circle high in the air
			drones.get(Nerve).hover(durationFinalCircle);
		//			drones.values().forEach(particle -> particle.moveHorizontalCircle(center, true, 0.07, durationFinalCircle)); //big circle high in the air
			
			//moveLandNerveAndFievel(drones, 6);
			List<DroneName> remainingDrones = Lists.newArrayList(Romeo, Dumbo, Juliet);
			List<DroneName> hoveringDrones = Lists.newArrayList(Nerve, Fievel);
			moveDronesToLineAndSpin(drones, remainingDrones, hoveringDrones, 30);
			
			drones.get(Romeo).moveToPoint(Point4D.create(3.0, 1.5, 0.8, YAW), 7);
			drones.get(Dumbo).moveToPoint(Point4D.create(5.0, 1.5, 0.8, YAW), 7);
			drones.get(Juliet).moveHorizontalCircle(center, false, 0.07, 14);
			drones.get(Juliet).moveToPoint(Point4D.create(5.5, 3.5, 0.8, YAW), 5);
			
			
//			drones.values().forEach(particle -> particle.hover(10));
		}
		
		private void moveDronesToLineAndSpin(Map<DroneName, Particle> drones, List<DroneName> remainingDrones, List<DroneName> hoveringDrones, double duration) {
			Map<DroneName, Particle> remainingDronesMap = Maps.newHashMap();
			remainingDrones.forEach(drone -> remainingDronesMap.put(drone, drones.get(drone)));

			double durationUp = 1;
			double durationDown = 3;
				double numSpacesBetweenDrones = 2;
				double numSpacesZ = 2;
				double minX = 2;
				double maxX = 4;
				double minY = 1.4;
				double maxY = 2.6;
				
				double minZ = 1.2;
				double maxZ = 2.4;
				
				double distX = (maxX - minX)/numSpacesBetweenDrones;
				double distY = (maxY - minY)/numSpacesBetweenDrones;
				double distZ = (maxZ - minZ)/numSpacesZ;
				
				//First
				Point4D a = Point4D.create(minX + 0 * distX, minY + 1 * distY, minZ + 2 * distZ, YAW); 
				Point4D b = Point4D.create(minX + 1 * distX, minY + 1 * distY, minZ + 1 * distZ, YAW); 
				Point4D c = Point4D.create(minX + 2 * distX, minY + 1 * distY, minZ + 0.01 * distZ, YAW); 
				
				double yPosition = 5;
				double height = 0.8;
				
				drones.get(Juliet).moveToPoint(a, durationDown);
				drones.get(Dumbo).moveToPoint(b, durationDown);
				drones.get(Romeo).moveToPoint(c, durationDown);
			
//			remainingDronesMap.get(Juliet).moveForwardToYPositionAtHeight(3, 1.3, durationUp);
//			remainingDronesMap.get(Dumbo).moveForwardToYPositionAtHeight(3, 1.7, durationUp);
//			remainingDronesMap.get(Romeo).moveForwardToYPositionAtHeight(3, 2.1, durationUp);
			
			
			remainingDronesMap.values().forEach(particle -> particle.moveHorizontalCircle(Point4D.create(2.8, 2.5, 3, YAW), 
					false, 
					0.06,
					duration - durationUp - durationDown,
					1,
					0));
			double durationFeivelMoveDown = 5;
			drones.get(Fievel).moveToPoint(Point4D.create(0.5, 4.5, 0.8, YAW), durationFeivelMoveDown);
			
			hoveringDrones.forEach(drone -> drones.get(drone).hover(duration));
		}
		
		
		private void moveLandNerveAndFievel(Map<DroneName, Particle> drones, int duration) {
			double yPosition = 5;
			double height = 1;
//			drones.get(Nerve).moveDownToHeight(height, duration/2);
//			drones.get(Fievel).moveDownToHeight(height, duration/2);
//			drones.get(Nerve).moveForwardToYPosition(yPosition, duration/2);
//			drones.get(Fievel).moveForwardToYPosition(yPosition, duration/2);
	
//			drones.get(Nerve).moveForwardToYPositionAtHeight(yPosition, height, duration);
//			drones.get(Fievel).moveForwardToYPositionAtHeight(yPosition, height, duration);
			
			drones.get(Nerve).hover(duration);
			drones.get(Dumbo).hover(duration);
			drones.get(Juliet).hover(duration);
			drones.get(Romeo).hover(duration);
		}

		/**
		 * @param drones
		 * @param duration
		 */
		private void moveToDiagonalLine(Map<DroneName, Particle> drones, double duration) {
			double numSpacesBetweenDrones = drones.size() - 1;
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
			
			moveDronesToPositionsAndHover(drones, a, b, c, d, e, duration);
			
			//second movement
			a = Point4D.create(minX + 0 * distX, minY + 4 * distY, maxZ - 4 * distZ, YAW); 
			b = Point4D.create(minX + 1 * distX, minY + 3 * distY, maxZ - 3 * distZ, YAW); 
			c = Point4D.create(minX + 2.001 * distX, minY + 2 * distY, maxZ - 2 * distZ, YAW); 
			d = Point4D.create(minX + 3 * distX, minY + 1 * distY, maxZ - 1 * distZ, YAW); 
			e = Point4D.create(minX + 4 * distX, minY + 0 * distY, maxZ - 0 * distZ, YAW); 
			
			moveDronesToPositionsAndHover(drones, a, b, c, d, e, duration);
		}
		
		
		private Point4D moveVerticalV(Map<DroneName, Particle> drones, double duration, double minX, double maxX) {
			double numSpacesBetweenDrones = drones.size() - 1; // hardcoding for 5 drones only
			double numSpacesZ = 2;
			double minY = 1.2;
			double maxY = 5;
			
			double minZ = 0.7;
			double maxZ = 3.0;
			
			double distX = (maxX - minX)/numSpacesBetweenDrones;
			double distY = (maxY - minY)/numSpacesBetweenDrones;
			double distZ = (maxZ - minZ)/numSpacesZ;
			
			//First
			Point4D a = Point4D.create(minX + 0 * distX, minY + 1 * distY, minZ + 2 * distZ, YAW); 
			Point4D b = Point4D.create(minX + 1 * distX, minY + 1 * distY, minZ + 1 * distZ, YAW); 
			Point4D c = Point4D.create(minX + 2 * distX, minY + 1 * distY, minZ + 0.01 * distZ, YAW); 
			Point4D d = Point4D.create(minX + 3 * distX, minY + 1 * distY, minZ + 1 * distZ, YAW); 
			Point4D e = Point4D.create(minX + 4 * distX, minY + 1 * distY, minZ + 2 * distZ, YAW); 
			
			double yPosition = 5;
			double height = 0.8;
			
			drones.get(Fievel).moveToPoint(a, duration);
			drones.get(Juliet).moveToPoint(b, duration);
			drones.get(Dumbo).moveToPoint(d, duration);
			drones.get(Romeo).moveToPoint(e, duration);
//			drones.get(Nerve).moveForwardToYPositionAtHeight(yPosition, height, duration);
			drones.get(Nerve).moveToPoint(Point4D.create(6, 5, 0.8, YAW), duration);
			
			drones.values().forEach(particle -> particle.hover(2));
			
			return c;
		}
		
		private void moveDronesToPositionsAndHover(Map<DroneName, Particle> drones, Point4D a, Point4D b, Point4D c, Point4D d, Point4D e, double duration) {

			if (drones.get(Fievel) != null) {
				drones.get(Fievel).moveToPoint(a, duration);
			}
			if (drones.get(Juliet) != null) {
				drones.get(Juliet).moveToPoint(b, duration);
			}
			if (drones.get(Dumbo) != null) {
					drones.get(Dumbo).moveToPoint(c, duration);
			}
			if (drones.get(Romeo) != null) {
					drones.get(Romeo).moveToPoint(d, duration);
			}
			if (drones.get(Nerve) != null) {
				drones.get(Nerve).moveToPoint(e, duration);
			}
			
			drones.values().forEach(particle -> particle.hover(2));
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
			//First movement
			//
			Point4D a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 0.1 * distZ, YAW); 
			Point4D b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
			Point4D c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 2 * distZ, YAW); 
			Point4D d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 3 * distZ, YAW); 
			Point4D e = Point4D.create(minX + 4 * distX, minY + 1.5 * distY, minZ + 4 * distZ, YAW); 
			
			moveDronesToPositionsAndHover(drones, a, b, c, d, e, duration);
			
			//
			//Second movement
			//
			a = Point4D.create(minX + 0 * distX, minY + 1.5 * distY, minZ + 4 * distZ, YAW); 
			b = Point4D.create(minX + 1 * distX, minY + 1.5 * distY, minZ + 3 * distZ, YAW); 
			c = Point4D.create(minX + 2 * distX, minY + 1.5 * distY, minZ + 2.01 * distZ, YAW); 
			d = Point4D.create(minX + 3 * distX, minY + 1.5 * distY, minZ + 1 * distZ, YAW); 
			e = Point4D.create(minX + 4 * distX, minY + 1.5 * distY, minZ + 0.01 * distZ, YAW); 

			moveDronesToPositionsAndHover(drones, a, b, c, d, e, duration);
		}
		private void moveToCirclePosition(Map<DroneName, Particle> drones, Point4D circleCenter, double circleRadius, double duration) {
			drones.get(Dumbo).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 0*Math.PI/5), duration);
			drones.get(Nerve).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 2*Math.PI/5), duration);
			drones.get(Romeo).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 4*Math.PI/5), duration);
			drones.get(Juliet).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 6*Math.PI/5), duration);
			drones.get(Fievel).moveToPoint(Point4D.pointAtAngle(circleCenter, circleRadius, 8*Math.PI/5), duration);
		}
		
		private void moveBackwardAndForward(Map<DroneName, Particle> drones) {
			final int numSpaceBetweenDrones = drones.size() - 1; 
			final double height = 2.3;
			final double durationMoveBackwards = 6;
			final double distanceForward = 3.5;
			final double durationForward = 7;
			final double minY = 0.5;
			final double minX = 0.3;
			final double maxX = 5.7;
			final double distX = (maxX - minX)/numSpaceBetweenDrones;
			
			Point4D a = Point4D.create(minX + 0 * distX, minY, height, YAW);
			Point4D b = Point4D.create(minX + 1 * distX, minY, height, YAW);
			Point4D c = Point4D.create(minX + 2 * distX, minY, height, YAW);
			Point4D d = Point4D.create(minX + 3 * distX, minY, height, YAW);
			Point4D e = Point4D.create(minX + 4 * distX, minY, height, YAW);
			
			moveDronesToPositionsAndHover(drones, a, b, c, d, e, durationMoveBackwards);
			
			drones.forEach((drone, particle) -> {
				if (drone == Juliet || drone == Romeo) {
					particle.moveForwardGoingHighInBetween(distanceForward, 1.25, 1.5, durationForward);
					particle.hover(5);

				} else {
					particle.moveForward(distanceForward, durationForward);
					particle.hover(5);
				}
			});
			
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
