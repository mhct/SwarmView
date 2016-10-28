/**
 * 
 */
package applications.trajectory;

import control.FiniteTrajectory4d;
import control.dto.Pose;

/**
 * @author tom
 *
 */
public class Hover implements FiniteTrajectory4d {

	private Pose pose;
	private double duration;

	public Hover (Pose pose, double duration) {
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
		return this.pose;
	}

}
