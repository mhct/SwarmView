package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

public class SpiralDecorator implements FiniteTrajectory4d {
	
	private FiniteTrajectory4d component;
	private double openingRate;
	private Point4D center;
	
	public SpiralDecorator(FiniteTrajectory4d component, Point4D center, double openingRate) {
		this.component = component;
		this.center = center;
		this.openingRate = openingRate;
	}
	
	@Override
	public double getTrajectoryDuration() {
		return component.getTrajectoryDuration();
	}
	
	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		Pose tempPose = component.getDesiredPosition(timeInSeconds);
		final double distance = openingRate * timeInSeconds;
		
		double dx = tempPose.x() - center.getX();
		double dy = tempPose.y() - center.getY();
		double dz = tempPose.z() - center.getZ();
		double modulus = Math.sqrt(dx*dx + dy*dy + dz*dz);
		
		double lambda = 0.0;
		if (Math.abs(modulus - 0.0) >= 0.00001) {
			lambda = (modulus + distance) / modulus;
			Point4D destination = Point4D.create(
					center.getX() + (dx * lambda),
					center.getY() + (dy * lambda),
					center.getZ() + (dz * lambda),
					tempPose.yaw());
			return destination.toPose();
		} else {
			return tempPose;
		}
	}
}
