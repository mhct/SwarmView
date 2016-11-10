package io.github.agentwise.swarmview.applications.trajectory;

import static com.google.common.base.Preconditions.checkArgument;

import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;

/**
 * A swing trajectory in 1 dimensions of motion specified in a frequency (How many revolutions per
 * second) and a radius (half of the total distance covered). This swing trajectory follows a
 * periodic angular enterVelocity function over the radian circle which manifests in slowed
 * movements
 * near the edges of the range of motion.
 * This 1D trajectory assumes the X-dimension of the (x,y,z) space.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
class PendulumSwingTrajectory1D extends PeriodicTrajectory implements Trajectory1d {
    private static final double MAXRANGE_VELOCITY_PERIODIC_PART = 0.649091;

    /**
     * Constructor
     *
     * @param radius    The length of the virtual pendulum string (or radius).
     * @param frequency The frequency f (amount of revolutions per second). Equals 1/period.
     */
    PendulumSwingTrajectory1D(double radius, double frequency) {
        this(Point4D.origin(), radius, frequency, 0);
    }

    PendulumSwingTrajectory1D(Point4D origin, double radius, double frequency, double phase) {
        super((HALFPI * 3) + phase, origin, radius, frequency);
        checkArgument(
                Math.abs(radius * frequency)
                        < MAX_ABSOLUTE_VELOCITY / (PISQUARED * MAXRANGE_VELOCITY_PERIODIC_PART),
                "Absolute speed should not be larger than "
                        + "MAX_ABSOLUTE_VELOCITY,"
                        + " which is: "
                        + MAX_ABSOLUTE_VELOCITY);
    }

    @Override
    public double getDesiredPosition(double timeInSeconds) {
        return getLinearDisplacement().getX()
                + getRadius()
                * StrictMath.cos(
                TrajectoryUtils.pendulumAngleFromTime(timeInSeconds, getFrequency())
                        + getPhaseDisplacement());
    }
}
