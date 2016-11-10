package io.github.agentwise.swarmview.rats.acts.introduction;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import io.github.agentwise.swarmview.applications.trajectory.Trajectories;
import io.github.agentwise.swarmview.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.control.dto.Pose;

import org.slf4j.LoggerFactory;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public abstract class TwinDrones {
    protected static final double orientation = -Math.PI / 2;
    protected final double operatingHeight = 1.0;
    protected final double endHeight = 1.5;
    protected final Point3D circleCenterPoint = Point3D.create(4, 4, operatingHeight);
    protected final double waitAtStation = 1.5;
    protected final double frequency = 1 / 2.32d; // ~max velocity achieved with f ~= 1/2.12 ~= 0.47
    protected final double revolutions = 3.5;
    protected final double circleTiming = (1 / frequency) * revolutions;
    protected final double circleRadius = 1;
    protected final double enterVelocity = 1;
    protected final double exitVelocity = 2;

    protected Point4D initPos;
    protected Point4D finalPos;

    protected double startIntroAt;

    // TODO: These two trajectories rely on the fact that the 'first leg' have the same duration
    //Kristof: yes they do... for symmetry it wouldn't look right if they didn't.

    TwinDrones(Pose initPos, Pose finalPos, double startTime) {
        //TODO remove this when logback config file is available.
        Logger root = (Logger) LoggerFactory.getLogger(Logger.ROOT_LOGGER_NAME);
        root.setLevel(Level.INFO);

        startIntroAt = Math.max(0.1, startTime);
        this.initPos = Point4D.from(initPos);
        this.finalPos = Point4D.from(finalPos);
        System.out.println("TwinDrone will strat at " + startIntroAt);
    }

    protected abstract FiniteTrajectory4d getTrajectory();

    public static FiniteTrajectory4d createRomeoTrajectory(Pose initPos, Pose finalPos,
            double startTime) {
        return new Romeo(initPos, finalPos, startTime).getTrajectory();
    }

    public static FiniteTrajectory4d createJulietTrajectory(Pose initPos, Pose finalPos,
            double startTime) {
        return new Juliet(initPos, finalPos, startTime).getTrajectory();
    }

    public static class Romeo extends TwinDrones {

        Romeo(Pose initPos, Pose finalPos, double startTime) {
            super(initPos, finalPos, startTime);
        }

        private final Point4D wp1 = Point4D.create(5, 4, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(3, 4, operatingHeight, orientation);
        private final double phaseToConnectStart = Math.PI;

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            FiniteTrajectory4d firstLeg = Trajectories
                    .newStraightLineTrajectory(this.initPos, wp1, enterVelocity);
            FiniteTrajectory4d lastLeg = Trajectories
                    .newStraightLineTrajectory(wp2, this.finalPos, exitVelocity);

            double height = 2;
            Point3D dest = Point3D.plus(circleCenterPoint, Point3D.create(0, 0, height));
            double vertSpeed = height / (circleTiming / 2);
            double updatedPhase = phaseToConnectStart + (revolutions * Math.PI); // 2PI * revs/2

            FiniteTrajectory4d corkscrewUp = Trajectories.corkscrewTrajectoryBuilder()
                    .setOrigin(Point4D.from(circleCenterPoint, orientation)).setDestination(dest)
                    .setSpeed(vertSpeed).setFrequency(-frequency)
                    .setPhase(phaseToConnectStart).setRadius(circleRadius).build();
            FiniteTrajectory4d corkscrewDown = Trajectories.corkscrewTrajectoryBuilder()
                    .setOrigin(Point4D.from(dest, orientation)).setDestination(circleCenterPoint)
                    .setSpeed(vertSpeed).setFrequency(frequency)
                    .setPhase(updatedPhase).setRadius(circleRadius).build();

            TrajectoryComposite choreo = TrajectoryComposite.builder()
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(this.initPos))
                    .withDuration(startIntroAt)
                    .addTrajectory(firstLeg)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(wp1))
                    .withDuration(waitAtStation)
                    .addTrajectory(corkscrewUp).withDuration(circleTiming / 2)
                    .addTrajectory(corkscrewDown).withDuration(circleTiming / 2)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(wp2))
                    .withDuration(waitAtStation).addTrajectory(lastLeg)
                    .build();

            LoggerFactory.getLogger(TwinDrones.class)
                    .info("Trajecory for Romeo takes " + choreo.getTrajectoryDuration()
                            + " seconds to perform fully.");
            return choreo;
        }
    }

    public static class Juliet extends TwinDrones {

        Juliet(Pose initPos, Pose finalPos, double startTime) {
            super(initPos, finalPos, startTime);
        }

        private final Point4D wp1 = Point4D.create(3, 4, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(5, 4, operatingHeight, orientation);
        private final double phaseToConnectStart = 0;

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            FiniteTrajectory4d firstLeg = Trajectories
                    .newStraightLineTrajectory(this.initPos, wp1, enterVelocity);
            FiniteTrajectory4d lastLeg = Trajectories
                    .newStraightLineTrajectory(wp2, this.finalPos, exitVelocity);

            double height = 2;
            Point3D dest = Point3D.plus(circleCenterPoint, Point3D.create(0, 0, height));
            double vertSpeed = height / (circleTiming / 2);
            double updatedPhase = phaseToConnectStart + (revolutions * Math.PI); // 2PI * revs/2

            FiniteTrajectory4d corkscrewUp = Trajectories.corkscrewTrajectoryBuilder()
                    .setOrigin(Point4D.from(circleCenterPoint, orientation)).setDestination(dest)
                    .setSpeed(vertSpeed).setFrequency(-frequency)
                    .setPhase(phaseToConnectStart).setRadius(circleRadius).build();
            FiniteTrajectory4d corkscrewDown = Trajectories.corkscrewTrajectoryBuilder()
                    .setOrigin(Point4D.from(dest, orientation)).setDestination(circleCenterPoint)
                    .setSpeed(vertSpeed).setFrequency(frequency)
                    .setPhase(updatedPhase).setRadius(circleRadius).build();

            TrajectoryComposite choreo = TrajectoryComposite.builder()
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(this.initPos))
                    .withDuration(startIntroAt).addTrajectory(firstLeg)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(wp1))
                    .withDuration(waitAtStation)
                    .addTrajectory(corkscrewUp).withDuration(circleTiming / 2)
                    .addTrajectory(corkscrewDown).withDuration(circleTiming / 2)
                    .addTrajectory(Trajectories.newHoldPositionTrajectory(wp2))
                    .withDuration(waitAtStation).addTrajectory(lastLeg)
                    .build();

            LoggerFactory.getLogger(TwinDrones.class)
                    .info("Trajecory for Juliet takes " + choreo.getTrajectoryDuration()
                            + " seconds to perform fully.");
            return choreo;
        }
    }
}
