package io.github.agentwise.swarmview.trajectory.swarmmovements;

import java.util.ArrayList;
import java.util.List;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.CircleTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * A Particle represents a drone and its possible movements.
 * All movement behaviours are created assuming the body frame of the particle.
 * Thus, invoking {@link #moveRight(double, double) moveRight} method will move the Particle to its right size.
 * 
 * X--C--X   C = front facing camera,  X = propeller
 *   | |
 * X-----X
 * 
 * TODO: Particle needs to have a drone movement model.. to decide if it can move as asked...
 * @author Mario h.c.t.
 * 
 */
public class Particle {
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
		if (Math.abs(dy - 0.0) >= 0.00001 || Math.abs(dx - 0.0) >= 0.00001) {
			double theta = Math.atan2(dy, dx);

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
			System.out.println("HOVER in circle");
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

	public void moveForward(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX(), current.getY()+distance, current.getZ(), 0.0);
		moveToPoint(destination, duration);
	}

	public void moveBackward(double distance, double duration) {
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
	
	private void addMovement(FiniteTrajectory4d trajectory) {
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