package io.github.agentwise.swarmview.applications.trajectory;

import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;

/**
 * Utility class for static utilities used in defining trajectories.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public final class TrajectoryUtils {
    private TrajectoryUtils() {
    }

    /**
     * @param currentTime The time point in the motion.
     * @param frequency   The frequency of the pendulum movement.
     * @return The angle of the pendulum string with the z-axis for a given time if the pendulum
     * moves
     * with given frequency.
     */
    public static double pendulumAngleFromTime(double currentTime, double frequency) {
        return PeriodicTrajectory.HALFPI
                * StrictMath.cos(PeriodicTrajectory.TWOPI * frequency * currentTime);
    }

    /**
     * Gets the one-dimensional trajectory linear x from a four-dimensional trajectory.
     *
     * @param trajectory4d the four-dimensional trajectory
     * @return an one-dimensional trajectory represents the trajectory in the x coordinate
     */
    public static Trajectory1d getTrajectoryLinearX(final Trajectory4d trajectory4d) {
        return new Trajectory1d() {
            @Override
            public double getDesiredPosition(double timeInSeconds) {
                return trajectory4d.getDesiredPositionX(timeInSeconds);
            }
        };
    }

    /**
     * Gets the one-dimensional trajectory linear y from a four-dimensional trajectory.
     *
     * @param trajectory4d the four-dimensional trajectory
     * @return an one-dimensional trajectory represents the trajectory in the y coordinate
     */
    public static Trajectory1d getTrajectoryLinearY(final Trajectory4d trajectory4d) {
        return new Trajectory1d() {
            @Override
            public double getDesiredPosition(double timeInSeconds) {
                return trajectory4d.getDesiredPositionY(timeInSeconds);
            }
        };
    }

    /**
     * Gets the one-dimensional trajectory linear z from a four-dimensional trajectory.
     *
     * @param trajectory4d the four-dimensional trajectory
     * @return an one-dimensional trajectory represents the trajectory in the z coordinate
     */
    public static Trajectory1d getTrajectoryLinearZ(final Trajectory4d trajectory4d) {
        return new Trajectory1d() {
            @Override
            public double getDesiredPosition(double timeInSeconds) {
                return trajectory4d.getDesiredPositionZ(timeInSeconds);
            }
        };
    }

    /**
     * Gets the one-dimensional trajectory angular z from a four-dimensional trajectory.
     *
     * @param trajectory4d the four-dimensional trajectory
     * @return an one-dimensional trajectory represents the trajectory in the angular z (the yaw)
     * coordinate
     */
    public static Trajectory1d getTrajectoryAngularZ(final Trajectory4d trajectory4d) {
        return new Trajectory1d() {
            @Override
            public double getDesiredPosition(double timeInSeconds) {
                return trajectory4d.getDesiredAngleZ(timeInSeconds);
            }
        };
    }

    /**
     * Sample a trajectory for a given time.
     *
     * @param trajectory the trajectory to sample.
     * @param time       the time point for which to sample a trajectory.
     * @return a point4D instance containing the sample location values.
     */
    public static Point4D sampleTrajectory(Trajectory4d trajectory, double time) {
        return Point4D.create(
                trajectory.getDesiredPositionX(time), trajectory.getDesiredPositionY(time),
                trajectory.getDesiredPositionZ(time), trajectory.getDesiredAngleZ(time));
    }

    /**
     * Sample a trajectory for a given time.
     *
     * @param trajectory the trajectory to sample.
     * @param time       the time point for which to sample a trajectory.
     * @return a point4D instance containing the sample location values.
     */
    public static Point4D sampleTrajectory(FiniteTrajectory4d trajectory, double time) {
    	return Point4D.create(
    			trajectory.getDesiredPosition(time).x(), trajectory.getDesiredPosition(time).y(),
    			trajectory.getDesiredPosition(time).z(), trajectory.getDesiredPosition(time).yaw());
    }
    
    /** 
     * Creates a wrapper instance of a FiniteTrajectory4d class, to translate it to a Trajectory4d class
     */
    public static Trajectory4d createFrom(FiniteTrajectory4d finiteTrajectory) {
    	return new Trajectory4d() {
			private final FiniteTrajectory4d trajectory = finiteTrajectory;
    		
			@Override
			public double getDesiredPositionZ(double timeInSeconds) {
				return trajectory.getDesiredPosition(timeInSeconds).z();
			}
			
			@Override
			public double getDesiredPositionY(double timeInSeconds) {
				return trajectory.getDesiredPosition(timeInSeconds).y();
			}
			
			@Override
			public double getDesiredPositionX(double timeInSeconds) {
				return trajectory.getDesiredPosition(timeInSeconds).x();
			}
			
			@Override
			public double getDesiredAngleZ(double timeInSeconds) {
				return trajectory.getDesiredPosition(timeInSeconds).yaw();
			}
		};
    }
     
}
