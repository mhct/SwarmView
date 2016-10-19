package show;

import applications.trajectory.Trajectories;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import choreo.Choreography;
import control.FiniteTrajectory4d;
import control.Trajectory4d;

/**
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class TwinDrones {
    protected static final double orientation = -Math.PI / 2;
    protected final double operatingHeight = 1.5;
    protected final Point3D centerPoint = Point3D.create(4, 5, operatingHeight);
    protected final double waitAtStation = 1;
    protected final double circleTiming = 3;

    TwinDrones() {
    }

    protected abstract Trajectory4d getTrajectory();

    public static Trajectory4d createRomeoTrajectory() {
        return new Romeo().getTrajectory();
    }

    public static Trajectory4d createJulietTrajectory() {
        return new Juliet().getTrajectory();
    }

    public static class Romeo extends TwinDrones {
        private final Point4D takeOff = Point4D.create(7, 5, operatingHeight, orientation);
        private final Point4D endPoint = Point4D.origin();
        private final Point4D wp1 = Point4D.create(5, 5, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(3, 5, operatingHeight, orientation);
        private final double velocity = 1.5;
        private final double phaseToConnectStart = Math.PI;
        private final double frequency = 1 / 5d;

        @Override
        protected Trajectory4d getTrajectory() {
            FiniteTrajectory4d firstLeg = Trajectories
                    .newStraightLineTrajectory(takeOff, wp1, velocity);
            FiniteTrajectory4d lastLeg = Trajectories
                    .newStraightLineTrajectory(wp2, endPoint, velocity);
            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder().fixYawAt(orientation)
                    .setFrequency(frequency).setPhase(phaseToConnectStart).setRadius(1)
                    .setLocation(centerPoint).build();

            return Choreography.builder().withTrajectory(firstLeg)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp1))
                    .forTime(waitAtStation).withTrajectory(circleTraj).forTime(circleTiming)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp2))
                    .forTime(waitAtStation).withTrajectory(lastLeg).build();
        }
    }

    public static class Juliet extends TwinDrones {
        private final Point4D takeOff = Point4D.create(1, 5, operatingHeight, orientation);
        private final Point4D endPoint = Point4D.origin();
        private final Point4D wp1 = Point4D.create(3, 5, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(5, 5, operatingHeight, orientation);
        private final double velocity = 1.5;
        private final double phaseToConnectStart = 0;
        private final double frequency = 1 / 5d;

        @Override
        protected Trajectory4d getTrajectory() {
            FiniteTrajectory4d firstLeg = Trajectories
                    .newStraightLineTrajectory(takeOff, wp1, velocity);
            FiniteTrajectory4d lastLeg = Trajectories
                    .newStraightLineTrajectory(wp2, endPoint, velocity);
            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder().fixYawAt(orientation)
                    .setFrequency(frequency).setPhase(phaseToConnectStart).setRadius(1)
                    .setLocation(centerPoint).build();

            return Choreography.builder().withTrajectory(firstLeg)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp1))
                    .forTime(waitAtStation).withTrajectory(circleTraj).forTime(circleTiming)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp2))
                    .forTime(waitAtStation).withTrajectory(lastLeg).build();
        }
    }
}
