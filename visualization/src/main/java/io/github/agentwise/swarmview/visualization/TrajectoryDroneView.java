package io.github.agentwise.swarmview.visualization;

import com.google.common.base.Preconditions;

import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

public class TrajectoryDroneView extends DroneView {
	private final FiniteTrajectory4d trajectory;
	
	TrajectoryDroneView(MultiDronesUI canvas, FiniteTrajectory4d trajectory, int color, int trailSize, String name) {
		super(canvas, color, trailSize, name);
		Preconditions.checkNotNull(color);
		Preconditions.checkArgument(trailSize >= 0 && trailSize <= 300);
		
		this.trajectory = trajectory;
	}
	
	TrajectoryDroneView(MultiDronesUI canvas, FiniteTrajectory4d trajectory) {
		this(canvas, trajectory, 255, 50, "");
	}

	Pose getDesiredPosition(float timeStep) {
		return trajectory.getDesiredPosition(timeStep);
	}
}
