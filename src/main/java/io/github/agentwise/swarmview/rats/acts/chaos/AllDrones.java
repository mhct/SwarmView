package io.github.agentwise.swarmview.rats.acts.chaos;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import io.github.agentwise.swarmview.applications.trajectory.Trajectories;
import io.github.agentwise.swarmview.applications.trajectory.Trajectory4d;
import io.github.agentwise.swarmview.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.control.dto.Pose;

import com.google.common.collect.Lists;

import org.slf4j.LoggerFactory;

import java.util.List;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public abstract class AllDrones {
    protected static final double orientation = -Math.PI / 2;
    protected final double actStartTime = 0;
    protected final double frequency = 1 / 2.32d; // ~max velocity achieved with f ~= 1/2.12 ~= 0.47
    protected final double revolutions = 3.0;
    protected final double circleTiming = (1 / frequency) * revolutions;
    protected final double circleRadius = 1;
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

    protected FiniteTrajectory4d getFirstLeg() {
        return TrajectoryComposite.builder().addTrajectory(Trajectories
                .newStraightLineTrajectory(getStartPoint(), getWayPoint1(), baseVelocity))
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint1()))
                .withDuration(waitAtStation + actStartTime).build();
    }

    protected FiniteTrajectory4d getSecondLeg() {
        return TrajectoryComposite.builder().addTrajectory(Trajectories
                .newStraightLineTrajectory(getWayPoint1(), getWayPoint2(), baseVelocity))
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint2()))
                .withDuration(waitAtStation).build();
    }

    protected FiniteTrajectory4d getThirdLeg() {
        return TrajectoryComposite.builder().addTrajectory(Trajectories
                .newStraightLineTrajectory(getWayPoint2(), getWayPoint3(), baseVelocity))
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint3()))
                .withDuration(waitAtStation).build();
    }

    protected FiniteTrajectory4d getFourthLeg() {
        return TrajectoryComposite.builder().addTrajectory(Trajectories
                .newStraightLineTrajectory(getWayPoint3(), getWayPoint4(), baseVelocity))
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint4()))
                .withDuration(waitAtStation).build();
    }

    protected FiniteTrajectory4d getLastLeg() {
        return TrajectoryComposite.builder().addTrajectory(Trajectories
                .newStraightLineTrajectory(getWayPoint4(), getEndPoint(), baseVelocity))
                .addTrajectory(Trajectories.newHoldPositionTrajectory(getEndPoint()))
                .withDuration(waitAtStation).build();
    }

    protected FiniteTrajectory4d compose(List<FiniteTrajectory4d> components) {
        TrajectoryComposite.Builder builder = TrajectoryComposite.builder();
        components.forEach(t -> builder.addTrajectory(t));
        return builder.build();
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
        private final double phaseToConnectStart = Math.PI;

        public Romeo(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            Point3D circleCenterPoint = Point3D
                    .plus(Point3D.project(getWayPoint1()), Point3D.create(1, 0, 0));
            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder().fixYawAt(orientation)
                    .setFrequency(frequency).setPhase(phaseToConnectStart).setRadius(circleRadius)
                    .setLocation(circleCenterPoint).build();

            FiniteTrajectory4d circle = TrajectoryComposite.builder().addTrajectory(circleTraj)
                    .withDuration(circleTiming)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint1()))
                    .withDuration(waitAtStation).build();
            return compose(Lists.newArrayList(getFirstLeg(), circle, getSecondLeg(), getThirdLeg(),
                    getFourthLeg(), getLastLeg()));
        }
    }

    public static class Juliet extends AllDrones {
        private final double phaseToConnectStart = -Math.PI / 2;

        public Juliet(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            Point3D circleCenterPoint = Point3D
                    .plus(Point3D.project(getWayPoint1()), Point3D.create(0, 1, 0));
            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder().fixYawAt(orientation)
                    .setFrequency(frequency).setPhase(phaseToConnectStart).setRadius(circleRadius)
                    .setLocation(circleCenterPoint).build();

            FiniteTrajectory4d circle = TrajectoryComposite.builder().addTrajectory(circleTraj)
                    .withDuration(circleTiming)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(getWayPoint1()))
                    .withDuration(waitAtStation).build();
            return compose(Lists.newArrayList(getFirstLeg(), circle, getSecondLeg(), getThirdLeg(),
                    getFourthLeg(), getLastLeg()));
        }
    }

    public static class Nerve extends AllDrones {
        private final double phaseToConnectStart = -Math.PI / 2d;

        public Nerve(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            Point3D circleCenterPoint = Point3D
                    .minus(Point3D.project(getEndPoint()), Point3D.create(0, 0.5, 0));
            //            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder()
            // .fixYawAt(orientation)
            //                    .setFrequency(frequency).setPhase(phaseToConnectStart)
            // .setRadius(circleRadius)
            //                    .setLocation(circleCenterPoint).build();
            Trajectory4d circleTraj = Trajectories.swingTrajectoryBuilder().setRadius(0.5)
                    .setFrequency(frequency).setOrigin(Point4D.from(circleCenterPoint, orientation))
                    .setXzPlaneAngle(Math.PI / 2d).build();

            FiniteTrajectory4d pendulum = TrajectoryComposite.builder().addTrajectory(circleTraj)
                    .withDuration(circleTiming)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(getEndPoint()))
                    .withDuration(waitAtStation).build();
            return compose(Lists.newArrayList(getFirstLeg(), getSecondLeg(), getThirdLeg(),
                    getFourthLeg(), getLastLeg(), pendulum));
        }
    }

    public static class Fievel extends AllDrones {

        public Fievel(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return compose(Lists.newArrayList(getFirstLeg(), getSecondLeg(), getThirdLeg(),
                    getFourthLeg(), getLastLeg()));

        }
    }

    public static class Dumbo extends AllDrones {

        public Dumbo(List<Pose> waypoints, double duration) {
            super(waypoints);
        }

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            return compose(Lists.newArrayList(getFirstLeg(), getSecondLeg(), getThirdLeg(),
                    getFourthLeg(), getLastLeg()));

        }

    }
}
