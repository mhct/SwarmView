package rats.acts.chaos;

import applications.trajectory.Trajectories;
import applications.trajectory.composites.TrajectoryComposite;
import applications.trajectory.geom.point.Point4D;
import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import control.FiniteTrajectory4d;
import control.dto.Pose;
import org.slf4j.LoggerFactory;

/**
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class AllDrones {
    protected static final double orientation = -Math.PI / 2;
    protected final double actStartTime = 0;
    protected final double frequency = 1 / 2.32d; // ~max velocity achieved with f ~= 1/2.12 ~= 0.47
    protected final double revolutions = 3.5;
    protected final double circleTiming = (1 / frequency) * revolutions;
    protected final double introEndTime = 15;
    protected final double actDuration = 45;
    protected final double baseVelocity = 1.5;
    protected final double waitAtStation = 2;

    AllDrones(double duration) {
        //TODO remove this when logback config file is available.
        Logger root = (Logger) LoggerFactory.getLogger(Logger.ROOT_LOGGER_NAME);
        root.setLevel(Level.INFO);
    }

    protected abstract FiniteTrajectory4d getTrajectory(); //todo refactor out

    public static FiniteTrajectory4d createRomeoTrajectory(Pose initialPosition, Pose finalPosition,
            double duration) {
        return new Romeo(initialPosition, finalPosition, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createJulietTrajectory(Pose initialPosition,
            Pose finalPosition, double duration) {
        return new Juliet(initialPosition, finalPosition, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createFievelTrajectory(Pose initialPosition,
            Pose finalPosition, double duration) {
        return new Fievel(initialPosition, finalPosition, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createDumboTrajectory(Pose initialPosition, Pose finalPosition,
            double duration) {
        return new Dumbo(initialPosition, finalPosition, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createNerveTrajectory(Pose initialPosition, Pose finalPosition,
            double duration) {
        return new Nerve(initialPosition, finalPosition, duration).getTrajectory();
    }

    protected FiniteTrajectory4d configTrajectory() {
        FiniteTrajectory4d firstLeg = Trajectories
                .newStraightLineTrajectory(getStartPoint(), getWayPoint1(), baseVelocity);
        FiniteTrajectory4d secondLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint1(), getWayPoint2(), baseVelocity);
        FiniteTrajectory4d thirdLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint2(), getWayPoint3(), baseVelocity);
        FiniteTrajectory4d fourthLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint2(), getWayPoint4(), baseVelocity);
        FiniteTrajectory4d lastLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint4(), getEndPoint(), baseVelocity);

        TrajectoryComposite choreo = TrajectoryComposite.builder()
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getStartPoint()))
                .withDuration(waitAtStation + actStartTime).addTrajectory(firstLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint1()))
                .withDuration(waitAtStation).addTrajectory(secondLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint2()))
                .withDuration(waitAtStation).addTrajectory(thirdLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint3()))
                .withDuration(waitAtStation).addTrajectory(fourthLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint4()))
                .withDuration(waitAtStation)

                .addTrajectory(lastLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getEndPoint()))
                .untillTotalDuration(waitAtStation + actStartTime + actDuration)
                .build();

        LoggerFactory.getLogger(AllDrones.class)
                .info("Trajecory for Romeo takes " + choreo.getTrajectoryDuration()
                        + " seconds to perform fully.");
        return choreo;
    }

    protected abstract Point4D getStartPoint();

    protected abstract Point4D getEndPoint();

    protected abstract Point4D getWayPoint1();

    protected abstract Point4D getWayPoint2();

    protected abstract Point4D getWayPoint3();

    protected abstract Point4D getWayPoint4();

    public static class Romeo extends AllDrones {
        private final Point4D takeOff = Point4D.create(7, 5, operatingHeight, orientation);
        private final Point4D endPoint = Point4D.create(1.1, 5, endHeight, orientation);
        private final Point4D wp1 = Point4D.create(5, 4, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(3, 4, operatingHeight, orientation);
        private final double phaseToConnectStart = 0;

        public Romeo(Pose initialPosition,
                Pose finalPosition, double duration) {
            super(duration);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }
    }

    public static class Juliet extends AllDrones {
        private final Point4D takeOff = Point4D.create(1, 5, operatingHeight, orientation);
        private final Point4D endPoint = Point4D.create(4.9, 5, endHeight, orientation);
        private final Point4D wp1 = Point4D.create(3, 4, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(5, 4, operatingHeight, orientation);
        private final double phaseToConnectStart = Math.PI;

        public Juliet(Pose initialPosition,
                Pose finalPosition, double duration) {
            super(duration);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }
    }

    public static class Nerve extends AllDrones {

        public Nerve(Pose initialPosition, Pose finalPosition, double duration) {
            super(duration);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return null;
        }
    }

    public static class Fievel extends AllDrones {

        public Fievel(Pose initialPosition, Pose finalPosition, double duration) {
            super(duration);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return null;
        }
    }

    public static class Dumbo extends AllDrones {

        public Dumbo(Pose initialPosition, Pose finalPosition, double duration) {
            super(duration);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return null;
        }
    }
}
