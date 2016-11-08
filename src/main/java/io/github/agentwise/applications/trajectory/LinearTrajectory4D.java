package io.github.agentwise.applications.trajectory;

import io.github.agentwise.applications.trajectory.geom.point.Point4D;

/**
 * A linear trajectory in four dimensions as a composite of four 1d linear components.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
class LinearTrajectory4D extends BasicTrajectory implements Trajectory4d {

    private final Trajectory1d linearX;
    private final Trajectory1d linearY;
    private final Trajectory1d linearZ;
    private final Trajectory1d angleZ;

    LinearTrajectory4D(Point4D startComponent, Point4D speedComponent) {
        this.linearX = new LinearTrajectory1D(startComponent.getX(), speedComponent.getX());
        this.linearY = new LinearTrajectory1D(startComponent.getY(), speedComponent.getY());
        this.linearZ = new LinearTrajectory1D(startComponent.getZ(), speedComponent.getZ());
        this.angleZ = new LinearTrajectory1D(startComponent.getAngle(), speedComponent.getAngle());
    }

    @Override
    public double getDesiredPositionX(double timeInSeconds) {
        return this.linearX.getDesiredPosition(timeInSeconds);
    }

    @Override
    public double getDesiredPositionY(double timeInSeconds) {
        return this.linearY.getDesiredPosition(timeInSeconds);
    }

    @Override
    public double getDesiredPositionZ(double timeInSeconds) {
        return this.linearZ.getDesiredPosition(timeInSeconds);
    }

    @Override
    public double getDesiredAngleZ(double timeInSeconds) {
        return this.angleZ.getDesiredPosition(timeInSeconds);
    }
}
