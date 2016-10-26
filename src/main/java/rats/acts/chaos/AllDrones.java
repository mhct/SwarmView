package rats.acts.chaos;

import applications.trajectory.Trajectories;
import applications.trajectory.composites.TrajectoryComposite;
import applications.trajectory.geom.point.Point4D;
import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import control.dto.Pose;
import org.slf4j.LoggerFactory;

import java.util.List;

import static com.google.common.base.Preconditions.checkArgument;

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
    private List<Pose> wayPoints;

    AllDrones(List<Pose> waypoints) {
        checkArgument(waypoints.size() == 6);
        this.wayPoints = Lists.newArrayList(waypoints);
        //TODO remove this when logback config file is available.
        Logger root = (Logger) LoggerFactory.getLogger(Logger.ROOT_LOGGER_NAME);
        root.setLevel(Level.INFO);
    }

    protected abstract FiniteTrajectory4d getTrajectory(); //todo refactor out

    public static FiniteTrajectory4d createRomeoTrajectory(List<Pose> waypoints, double duration) {
        return new Romeo(waypoints, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createJulietTrajectory(List<Pose> waypoints, double duration) {
        return new Juliet(waypoints, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createFievelTrajectory(List<Pose> waypoints, double duration) {
        return new Fievel(waypoints, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createDumboTrajectory(List<Pose> waypoints, double duration) {
        return new Dumbo(waypoints, duration).getTrajectory();
    }

    public static FiniteTrajectory4d createNerveTrajectory(List<Pose> waypoints, double duration) {
        return new Nerve(waypoints, duration).getTrajectory();
    }

    protected FiniteTrajectory4d configTrajectory() {
        FiniteTrajectory4d firstLeg = Trajectories
                .newStraightLineTrajectory(getStartPoint(), getWayPoint1(), baseVelocity);
        FiniteTrajectory4d secondLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint1(), getWayPoint2(), baseVelocity);
        FiniteTrajectory4d thirdLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint2(), getWayPoint3(), baseVelocity);
        FiniteTrajectory4d fourthLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint3(), getWayPoint4(), baseVelocity);
        FiniteTrajectory4d lastLeg = Trajectories
                .newStraightLineTrajectory(getWayPoint4(), getEndPoint(), baseVelocity);

        TrajectoryComposite choreo = TrajectoryComposite.builder()
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getStartPoint()))
                .withDuration(waitAtStation + actStartTime)
                .addTrajectory(firstLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint1()))
                .withDuration(waitAtStation)
                .addTrajectory(secondLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint2()))
                .withDuration(waitAtStation)
                .addTrajectory(thirdLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint3()))
                .withDuration(waitAtStation)
                .addTrajectory(fourthLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint4()))
                .withDuration(waitAtStation)
                .addTrajectory(lastLeg)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getEndPoint()))
                //                .untillTotalDuration(waitAtStation + actStartTime + actDuration)
                .withDuration(waitAtStation)
                .build();

        LoggerFactory.getLogger(AllDrones.class)
                .info("Trajecory for this drone takes " + choreo.getTrajectoryDuration()
                        + " seconds to perform fully.");
        return choreo;
    }

    protected Point4D getStartPoint() {
        return Point4D.from(wayPoints.get(0));
    }

    protected Point4D getEndPoint() {
        return Point4D.from(wayPoints.get(5));
    }

    protected Point4D getWayPoint1() {
        return Point4D.from(wayPoints.get(1));
    }

    protected Point4D getWayPoint2() {
        return Point4D.from(wayPoints.get(2));
    }

    protected Point4D getWayPoint3() {
        return Point4D.from(wayPoints.get(3));
    }

    protected Point4D getWayPoint4() {
        return Point4D.from(wayPoints.get(4));
    }

    public static class Romeo extends AllDrones {

        public Romeo(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }
    }

    public static class Juliet extends AllDrones {

        public Juliet(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }
    }

    public static class Nerve extends AllDrones {

        public Nerve(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }
    }

    public static class Fievel extends AllDrones {

        public Fievel(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }
    }

    public static class Dumbo extends AllDrones {

        public Dumbo(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return configTrajectory();
        }

    }
}
