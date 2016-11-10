package io.github.agentwise.swarmview.applications.trajectory;

/**
 * A linear trajectory in one dimension.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
class LinearTrajectory1D extends BasicTrajectory implements Trajectory1d {
    private final double startComp;
    private final double speedComp;

    LinearTrajectory1D(double startComponent, double speedComponent) {
        this.startComp = startComponent;
        this.speedComp = speedComponent;
    }

    @Override
    public double getDesiredPosition(double timeInSeconds) {
        return startComp + speedComp * timeInSeconds;
    }
}
