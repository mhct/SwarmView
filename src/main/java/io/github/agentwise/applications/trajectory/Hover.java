/**
 * 
 */
package io.github.agentwise.applications.trajectory;

import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;

/**
 * @author tom
 *
 */
public class Hover implements FiniteTrajectory4d {

	private Point4D pose;
	private double duration;

	public Hover (Pose pose, double duration) {
		this(Point4D.from(pose), duration);
	}

	public Hover (Point4D pose, double duration) {
		this.pose = pose;
		this.duration = duration;
	}

	/* (non-Javadoc)
	 * @see control.FiniteTrajectory4d#getTrajectoryDuration()
	 */
	@Override
	public double getTrajectoryDuration() {
		// TODO Auto-generated method stub
		return this.duration;
	}

	/* (non-Javadoc)
	 * @see control.FiniteTrajectory4d#getDesiredPosition(double)
	 */
	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		if (timeInSeconds < 0) {
			throw new IllegalArgumentException ("Pose for time < 0 is not allowed ("+timeInSeconds+")");
		}
		return Pose.create(this.pose.getX(), this.pose.getY(), this.pose.getZ(), this.pose.getAngle());
	}

}
