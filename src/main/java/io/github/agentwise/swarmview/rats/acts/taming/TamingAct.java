package io.github.agentwise.rats.acts.taming;

import static io.github.agentwise.control.DroneName.Dumbo;
import static io.github.agentwise.control.DroneName.Fievel;
import static io.github.agentwise.control.DroneName.Juliet;
import static io.github.agentwise.control.DroneName.Nerve;
import static io.github.agentwise.control.DroneName.Romeo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import io.github.agentwise.applications.trajectory.CircleTrajectory4D;
import io.github.agentwise.applications.trajectory.Hover;
import io.github.agentwise.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.Act;
import io.github.agentwise.control.ActConfiguration;
import io.github.agentwise.control.DroneName;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;
import io.github.agentwise.rats.acts.interact.InterAct;

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

		
		//
		// Go to initial positions
		//
		Map<DroneName, Pose> beginMovement = new LinkedHashMap<DroneName, Pose>(5);
		beginMovement.put(Nerve, Pose.create(5.0, 2.0, 1.5, 0));
		beginMovement.put(Romeo, Pose.create(3.0, 2.0, 1.5, 0));
		beginMovement.put(Fievel, Pose.create(4, 6.0, 1.5, 0));
		beginMovement.put(Dumbo, Pose.create(6.2, 4.5, 1.5, 0));
		beginMovement.put(Juliet, Pose.create(1.8, 4.5, 1.5, 0));
		
		List<DroneName> beginMovementOrder = Arrays.asList(Nerve, Romeo, Fievel, Dumbo, Juliet);
		ActConfiguration beginActConfiguration = ActConfiguration.createFromInitialFinalPositions(act.initialPositions(), beginMovement);
		Act goToPosition = InterAct.createWithOrderedSequentialMovement(beginActConfiguration, 1.0, beginMovementOrder);
		goToPosition.lockAndBuild();
		
		//
		// Perform the joint movements
		//
		Swarm swarm = new Swarm(beginActConfiguration.finalPositionConfiguration());
		swarm.script();

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
	
	
	static class SwarmAct extends Act {
		private Swarm swarm;
		
		public SwarmAct(ActConfiguration configuration) {
			super(configuration);
			swarm = new Swarm(configuration.initialPositionConfiguration());
			swarm.script();
		}
		
	}
	
	static class Swarm {
		Map<DroneName, Particle> drones;
		
		public Swarm(Map<DroneName, Pose> initialConfiguration) {
			drones = new LinkedHashMap<>();
			for (Map.Entry<DroneName, Pose> entry: initialConfiguration.entrySet()) {
				drones.put(entry.getKey(), new Particle(entry.getValue()));
			}
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
				drones.values().forEach(drone -> drone.moveNorth(distance, duration));
				drones.values().forEach(drone -> drone.moveLeft(distance, duration));
				drones.values().forEach(drone -> drone.moveSouth(distance, duration));
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
//			drones.values().forEach(drone -> drone.moveCircle(center, true, 10));
//			drones.values().forEach(drone -> drone.moveCircle(center, false, 10));
			
		}

	}
	
	// Particle needs to have a drone movement model.. to decide if it can move as asked...
	static class Particle {
		private List<FiniteTrajectory4d> movementParts;
		private Point4D current;
		
		public Particle(Pose initial) {
			this.current = Point4D.from(initial);
			movementParts = new ArrayList<>();
		}

		public void moveCircle(Point4D center, boolean clockwise, double duration) {
			double frequency;
			if (clockwise) {
				frequency = 0.1;
			} else {
				frequency = -0.1;
			}
			double dx = current.getX() - center.getX();
			double dy = current.getY() - center.getY();
			double distanceToCenter = Math.sqrt(dx*dx+dy*dy);
			if (Math.abs(dy - 0.0) >= 0.00001) {
				double theta = Math.acos(dx/(distanceToCenter));
				if (dy <= 0.0) {
					theta += Math.PI;
				} 
				
				FiniteTrajectory4d circle = TrajectoryComposite.builder().addTrajectory(
						CircleTrajectory4D.builder()
						.setLocation(Point3D.project(center))
						.setPhase(theta)
						.fixYawAt(-Math.PI/2)
						.setRadius(distanceToCenter)
						.setFrequency(frequency)
						.build()).withDuration(duration).build();
				this.addMovement(circle);
			} else {
				this.addMovement(new Hover(current, duration));
			}
		}

		public void moveDown(double distance, double duration) {
			Point4D destination = Point4D.create(current.getX(), current.getY(), current.getZ() - distance, 0.0);
			moveToPoint(destination, duration);
		}

		public void moveUp(double distance, double duration) {
			Point4D destination = Point4D.create(current.getX(), current.getY(), current.getZ() + distance, 0.0);
			moveToPoint(destination, duration);
		}
		
		public void moveRight(double distance, double duration) {
			Point4D destination = Point4D.create(current.getX()-distance, current.getY(), current.getZ(), 0.0);
			moveToPoint(destination, duration);
		}

		public void moveLeft(double distance, double duration) {
			Point4D destination = Point4D.create(current.getX()+distance, current.getY(), current.getZ(), 0.0);
			moveToPoint(destination, duration);
		}

		public void moveNorth(double distance, double duration) {
			Point4D destination = Point4D.create(current.getX(), current.getY()+distance, current.getZ(), 0.0);
			moveToPoint(destination, duration);
		}

		public void moveSouth(double distance, double duration) {
			Point4D destination = Point4D.create(current.getX(), current.getY()-distance, current.getZ(), 0.0);
			moveToPoint(destination, duration);
		}

		private void moveToPoint(Point4D destination, double duration) {
			this.addMovement(StraightLineTrajectory4D.createWithCustomTravelDuration(current, destination, duration));
		}
		
		public void moveAway(Point4D center, double distance, double duration) {
			double dx = current.getX() - center.getX();
			double dy = current.getY() - center.getY();
			double dz = current.getZ() - center.getZ();
			double modulus = Math.sqrt(dx*dx + dy*dy + dz*dz);
			
			double lambda = 0.0;
			if (Math.abs(modulus - 0.0) >= 0.00001) {
				lambda = (modulus + distance) / modulus;
				Point4D destination = Point4D.create(
						center.getX() + (dx * lambda),
						center.getY() + (dy * lambda),
						center.getZ() + (dz * lambda),
						0.0);
				this.addMovement(StraightLineTrajectory4D.createWithCustomTravelDuration(current, destination, duration));
			} else {
				this.addMovement(new Hover(current, duration));
			}
			
		}

		public Point4D currentPoint() {
			return current;
		}
		
		public void addMovement(FiniteTrajectory4d trajectory) {
			movementParts.add(trajectory);
			current = Point4D.from(trajectory.getDesiredPosition(trajectory.getTrajectoryDuration()));
		}
		
		//compose the trajectories
		public FiniteTrajectory4d getTrajectory() {
			Builder builder = TrajectoryComposite.builder();
			for (FiniteTrajectory4d part: movementParts) {
				builder.addTrajectory(part);
			}
			
			return builder.build();
		}

	}
}
