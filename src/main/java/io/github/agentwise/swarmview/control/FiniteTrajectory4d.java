package io.github.agentwise.control;

import io.github.agentwise.control.dto.Pose;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public interface FiniteTrajectory4d {
    /**
     * @return The estimated point in time after consuming this trajectory, that the trajectory will
     * not perform any more movement.
     */
    double getTrajectoryDuration();

    /**
     * Returns the desired position of a drone
     *
     * @param timeInSeconds
     * @return Pose of the drone
     */
    Pose getDesiredPosition(double timeInSeconds);
}
