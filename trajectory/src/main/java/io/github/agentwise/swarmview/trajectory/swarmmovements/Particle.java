package io.github.agentwise.swarmview.trajectory.swarmmovements;

import static com.google.common.base.Preconditions.checkArgument;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.CircleTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.Trajectory4d;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.WiggleTrajectory;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.ZigZagTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.VerticalMovementDecorator;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.SineVerticalDecorator;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.SpiralDecorator;

/**
 * A Particle represents a drone and its possible movements. All movement behaviours are created
 * assuming the body frame of the particle. Thus, invoking {@link #moveRight(double, double)
 * moveRight} method will move the Particle to its right size.
 *
 * <p>X--C--X C = front facing camera, X = propeller | | X-----X
 *
 * <p>TODO: Particle needs to have a drone movement model.. to decide if it can move as asked...
 *
 * @author Mario h.c.t.
 */
public class Particle {
	private List<FiniteTrajectory4d> movementParts;
	private Point4D current;
	private static double YAW = -Math.PI / 2;
	private static final Random RANDOM_GENERATOR = new Random(123);

	public Particle(Pose initial) {
		this.current = Point4D.from(initial);
		movementParts = new ArrayList<>();
	}


	public void moveHorizontalCircle(Point4D center, boolean clockwise, double duration) {
		moveHorizontalCircle(center, clockwise, duration, 0, 0, false);
	}

	public void moveHorizontalCircle(Point4D center, boolean clockwise, double duration, double waveHeight, double openingRate) {
		moveHorizontalCircle(center, clockwise, duration, waveHeight, openingRate, false);
	}


	private void moveHorizontalCircle(Point4D center, boolean clockwise, double duration, double waveHeight, double openingRate, boolean corkscrew) {
		double frequency;
		if (clockwise) {
			frequency = 0.1;
		} else {
			frequency = -0.1;
		}
		double dx = current.getX() - center.getX();
		double dy = current.getY() - center.getY();
//		double dz = current.getZ() - center.getZ();

		double distanceToCenter = Math.sqrt(dx*dx+dy*dy);
		if (Math.abs(dy - 0.0) >= 0.00001 || Math.abs(dx - 0.0) >= 0.00001) {
			double theta = Math.atan2(dy, dx);
//			double gamma = Math.atan2(dz, distanceToCenter);

			FiniteTrajectory4d circle = TrajectoryComposite.builder().addTrajectory(
					CircleTrajectory4D.builder()
					.setLocation(Point3D.project(center))
					.setPhase(theta)
					.fixYawAt(YAW)
					.setRadius(distanceToCenter)
					.setFrequency(frequency)
					.build()).withDuration(duration).build();
			if (waveHeight > 0.0) {
				this.addMovement(new SineVerticalDecorator(circle, waveHeight));
			} else if (openingRate != 0.0) {
				this.addMovement(new SpiralDecorator(circle, center, openingRate));
			} else {
				this.addMovement(circle);
			}
		} else {
			System.out.println("HOVER in circle");
			this.addMovement(new Hover(current, duration));
		}
	}

	public void moveVerticalCorkscrew(Point4D center, double circleFrequency, double height, double duration) {
		final FiniteTrajectory4d horizontalCircle = TrajectoryComposite.builder()
				.addTrajectory(createHorizontalCircleTrajectory(current, center, circleFrequency))
				.withDuration(duration).build();
		this.addMovement(VerticalMovementDecorator.create(horizontalCircle, (height - current.getZ()) / duration));
	}

	private static Trajectory4d createHorizontalCircleTrajectory(Point4D currentPoint, Point4D center, double frequency) {
		double dx = currentPoint.getX() - center.getX();
		double dy = currentPoint.getY() - center.getY();
		double distanceToCenter = Math.sqrt(dx*dx+dy*dy);
		if (Math.abs(dy - 0.0) >= 0.00001 || Math.abs(dx - 0.0) >= 0.00001) {
			double theta = Math.atan2(dy, dx);
//			double gamma = Math.atan2(dz, distanceToCenter);

			return CircleTrajectory4D.builder()
							.setLocation(Point3D.create(center.getX(), center.getY(), currentPoint.getZ()))
							.setPhase(theta)
							.fixYawAt(YAW)
							.setRadius(distanceToCenter)
							.setFrequency(frequency)
							.build();
		} else {
			throw new RuntimeException("Infeasible circle.");
		}
	}

	public void moveDown(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX(), current.getY(), current.getZ() - distance, YAW);
		moveToPoint(destination, duration);
	}

	public void moveUp(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX(), current.getY(), current.getZ() + distance, YAW);
		moveToPoint(destination, duration);
	}

	public void moveRight(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX()-distance, current.getY(), current.getZ(), YAW);
		moveToPoint(destination, duration);
	}

	public void moveLeft(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX()+distance, current.getY(), current.getZ(), YAW);
		moveToPoint(destination, duration);
	}

	public void moveForward(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX(), current.getY()+distance, current.getZ(), YAW);
		moveToPoint(destination, duration);
	}

	public void moveBackward(double distance, double duration) {
		Point4D destination = Point4D.create(current.getX(), current.getY()-distance, current.getZ(), YAW);
		moveToPoint(destination, duration);
	}

	public void moveToPoint(Point4D destination, double duration) {
		this.addMovement(StraightLineTrajectory4D.createWithCustomTravelDuration(current, destination, duration));
	}

	public void moveToPointWithVelocity(Point4D destination, double velocity) {
		this.addMovement(StraightLineTrajectory4D.createWithCustomVelocity(current, destination, velocity));
	}

	public void moveZigZagToPoint(Point4D destination, double percentageVelocity) {
		final int numberOfZigZags = (int) (Point3D.distance(Point3D.project(destination), Point3D.project(current)) / 0.25);
		final double distanceOfZigZag = 5.0;
		this.addMovement(new ZigZagTrajectory4D(current, destination, numberOfZigZags, distanceOfZigZag, percentageVelocity));
	}

	public void wiggle(int numberOfWiggles, double timeToStayAtEdge) {
		this.addMovement(new WiggleTrajectory(currentPoint(), numberOfWiggles, timeToStayAtEdge));
	}

	public void rotateToAngle(double destinationAngle, double duration) {
		final Point4D desiredPoint = Point4D.create(current.getX(), current.getY(), current.getZ(), destinationAngle);
		this.addMovement(new Hover(desiredPoint, duration));
	}

	public void hover(double duration) {
		this.addMovement(new Hover(current, duration));
	}

	public void moveTriangleToPoint(Point4D destination, double heightOfMiddlePoint, double velocity) {
		final Point4D middlePoint =
				Point4D.create(
						current.getX() + (destination.getX() - current.getX()) / 2,
						current.getY() + (destination.getY() - current.getY()) / 2,
						heightOfMiddlePoint,
						current.getAngle() + (destination.getAngle() - current.getAngle()) / 2);

		moveToPointWithVelocity(middlePoint, velocity);
		moveToPointWithVelocity(destination, velocity);
	}
	public void moveTowardPointAndStopRandomlyBeforeReachingPoint(
		      Point4D destination,
		      double stoppingDistanceToDestinationLowerBound,
		      double stoppingDistanceToDestinationUpperBound,
		      double duration) {
		checkArgument(
		        stoppingDistanceToDestinationLowerBound >= 0,
		        "stopping distance must be greater than zero to ensure that the drone never more further than the destination");
		checkArgument(
				stoppingDistanceToDestinationUpperBound >= 0,
				"stopping distance must be greater than zero to ensure that the drone never more further than the destination");
		checkArgument(
				stoppingDistanceToDestinationUpperBound >= stoppingDistanceToDestinationLowerBound);

		final double stoppingDistanceToDestination =
			stoppingDistanceToDestinationLowerBound
	        + (stoppingDistanceToDestinationUpperBound - stoppingDistanceToDestinationLowerBound)
	            * RANDOM_GENERATOR.nextDouble();

		final Point3D lineVector =
			Point3D.minus(Point3D.project(destination), Point3D.project(current));
		final double normValueOfLineVector = lineVector.norm();
		final Point3D normVector = Point3D.scale(lineVector, 1 / normValueOfLineVector);
		final Point3D stoppingPoint =
			Point3D.minus(
	        Point3D.project(destination), Point3D.scale(normVector, stoppingDistanceToDestination));
		moveToPoint(Point4D.from(stoppingPoint, destination.getAngle()), duration);
	 }

	public void moveTowardPointAndStopRandomlyWithInRange(Point4D destination, double range,
			double duration) {
		final double rangeX = -range + 2 * range * RANDOM_GENERATOR.nextDouble();
		final double rangeY = -range + 2 * range * RANDOM_GENERATOR.nextDouble();
		final double rangeZ = -range + 2 * range * RANDOM_GENERATOR.nextDouble();
		final Point4D stoppingPosition = Point4D.create(destination.getX() + rangeX, destination.getY() + rangeY, destination.getZ() + rangeZ, destination.getAngle());
		moveToPoint(stoppingPosition, duration);
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
					YAW);
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

  