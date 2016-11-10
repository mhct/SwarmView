package io.github.agentwise.swarmview.applications.trajectory;

import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;

/**
 * Basic trajectories to be executed and synced at the time set by the user.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public abstract class BasicTrajectory {
    public static final double MAX_ABSOLUTE_VELOCITY = 3;
    protected final Point4D linearDisplacement;

    protected BasicTrajectory(Point4D displacement) {
        this.linearDisplacement = displacement;
    }

    protected BasicTrajectory() {
        this.linearDisplacement = Point4D.origin();
    }

    /**
     * Return the origin point for linear displacement in 4D.
     *
     * @return
     */
    protected Point4D getLinearDisplacement() {
        return this.linearDisplacement;
    }
}
