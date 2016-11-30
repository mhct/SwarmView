package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

public class SineVerticalDecorator implements FiniteTrajectory4d {

	private FiniteTrajectory4d component;
	private double halfAmplitude;

	@Override
	public double getTrajectoryDuration() {
		return component.getTrajectoryDuration();
	}

	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		Pose tempPose = component.getDesiredPosition(timeInSeconds);
		double z;
		z = tempPose.z() + (halfAmplitude + halfAmplitude * Math.sin(timeInSeconds - Math.PI/2));
		Pose pose = Pose.create(tempPose.x(), tempPose.y(), z, tempPose.yaw());
		
		return pose;
	}
	
	public SineVerticalDecorator(FiniteTrajectory4d component, double amplitude) {
		this.component = component;
		this.halfAmplitude = amplitude/2;
	}
	
}
