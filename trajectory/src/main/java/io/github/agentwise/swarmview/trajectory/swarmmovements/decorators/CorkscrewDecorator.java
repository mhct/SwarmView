package io.github.agentwise.swarmview.trajectory.swarmmovements.decorators;

import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

public class CorkscrewDecorator implements FiniteTrajectory4d {

	public CorkscrewDecorator(FiniteTrajectory4d trajectory, Point4D center, double rate) {
		
	}
	
	@Override
	public double getTrajectoryDuration() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		// TODO Auto-generated method stub
		return null;
	}

}
