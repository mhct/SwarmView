package io.github.agentwise.swarmview.trajectory.control;

import java.util.List;

public interface ChoreographyView {
	public int getNumberDrones();
	public FiniteTrajectory4d getFullTrajectory(DroneName name);
	public double getChoreographyDuration();
	public String getCurrentActName(float timeStep);
    public List<FiniteTrajectory4d> getAllTrajectories();
	public List<DroneName> getDroneNames();
}
