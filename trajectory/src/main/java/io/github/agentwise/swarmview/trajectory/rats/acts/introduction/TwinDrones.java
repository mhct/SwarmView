package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.Trajectories;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

import org.slf4j.LoggerFactory;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public class TwinDrones {
    protected final double orientation = -Math.PI / 2;
    protected final double operatingHeight = 1.0;
    protected final double endHeight = 1.5;
    protected final Point3D circleCenterPoint = Point3D.create(4, 4, operatingHeight);
    protected final double waitAtStation = 0.5;
    protected final double frequency = 1 / 2.32d; // ~max velocity achieved with f ~= 1/2.12 ~= 0.47
    protected final double revolutions = 3.5;
    protected final double circleTiming = (1 / frequency) * revolutions;
    protected final double circleRadius = 1;
    protected final double enterVelocity = 1;
    protected final double exitVelocity = 2;

    protected Point4D initPosRomeo;
    protected Point4D finalPosRomeo;
    protected Point4D initPosJuliet;
    protected Point4D finalPosJuliet;
    
	private final Point4D wp1Romeo = Point4D.create(5, 4, operatingHeight, orientation);
    private final Point4D wp2Romeo = Point4D.create(3, 4, operatingHeight, orientation);
    private final double phaseToConnectStartRomeo = Math.PI;
	private FiniteTrajectory4d firstLegRomeo;
    
    private final Point4D wp1Juliet = Point4D.create(3, 4, operatingHeight, orientation);
    private final Point4D wp2Juliet = Point4D.create(5, 4, operatingHeight, orientation);
    private final double phaseToConnectStartJuliet = 0;
	private FiniteTrajectory4d firstLegJuliet;

    protected double startIntroAt;

    // TODO: These two trajectories rely on the fact that the 'first leg' has the same duration
    //Kristof: yes they do... for symmetry it wouldn't look right if they didn't.

    public TwinDrones(Pose initPosRomeo, Pose finalPosRomeo, Pose initPosJuliet, Pose finalPosJuliet, double startTime) {
        //TODO remove this when logback config file is available.
        Logger root = (Logger) LoggerFactory.getLogger(Logger.ROOT_LOGGER_NAME);
        root.setLevel(Level.INFO);

        startIntroAt = Math.max(0.1, startTime);
        this.initPosRomeo = Point4D.from(initPosRomeo);
        this.finalPosRomeo = Point4D.from(finalPosRomeo);
        this.initPosJuliet = Point4D.from(initPosJuliet);
        this.finalPosJuliet = Point4D.from(finalPosJuliet);
        
        this.firstLegRomeo = Trajectories
                .newStraightLineTrajectory(this.initPosRomeo, wp1Romeo, enterVelocity);
        this.firstLegJuliet = Trajectories
                .newStraightLineTrajectory(this.initPosJuliet, wp1Juliet, enterVelocity);
        // System.out.println("TwinDrone will strat at " + startIntroAt);
    }

    public FiniteTrajectory4d getRomeoTrajectory() {

    	// deal with first leg, and synchronize before corkscrew
    	double romeoDurationFirstLeg = this.firstLegRomeo.getTrajectoryDuration();
    	double julietDurationFirstLeg = this.firstLegJuliet.getTrajectoryDuration();
    	double waitToSynchroize = 0;
    	if (romeoDurationFirstLeg < julietDurationFirstLeg) {
    		waitToSynchroize = julietDurationFirstLeg - romeoDurationFirstLeg;
    	}
    	double waitBeforeCorkScrew = waitToSynchroize + waitAtStation;
    	
    	// deal with last leg
        FiniteTrajectory4d lastLeg = Trajectories
                .newStraightLineTrajectory(this.wp2Romeo, this.finalPosRomeo, exitVelocity);

        // corkscrew
        double height = 2;
        Point3D dest = Point3D.plus(circleCenterPoint, Point3D.create(0, 0, height));
        double vertSpeed = height / (circleTiming / 2);
        double updatedPhase = phaseToConnectStartRomeo + (revolutions * Math.PI); // 2PI * revs/2

        FiniteTrajectory4d corkscrewUp = Trajectories.corkscrewTrajectoryBuilder()
                .setOrigin(Point4D.from(circleCenterPoint, orientation)).setDestination(dest)
                .setSpeed(vertSpeed).setFrequency(-frequency)
                .setPhase(phaseToConnectStartRomeo).setRadius(circleRadius).build();
        FiniteTrajectory4d corkscrewDown = Trajectories.corkscrewTrajectoryBuilder()
                .setOrigin(Point4D.from(dest, orientation)).setDestination(circleCenterPoint)
                .setSpeed(vertSpeed).setFrequency(frequency)
                .setPhase(updatedPhase).setRadius(circleRadius).build();

        // construct trajectory
        TrajectoryComposite choreo = TrajectoryComposite.builder()
                .addTrajectory(Trajectories.newHoldPositionTrajectory(this.initPosRomeo))
                .withDuration(startIntroAt)
                .addTrajectory(this.firstLegRomeo)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(wp1Romeo))
                .withDuration(waitBeforeCorkScrew)
                .addTrajectory(corkscrewUp).withDuration(circleTiming / 2)
                .addTrajectory(corkscrewDown).withDuration(circleTiming / 2)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(wp2Romeo))
                .withDuration(waitAtStation).addTrajectory(lastLeg)
                .build();

        LoggerFactory.getLogger(TwinDrones.class)
                .info("Trajecory for Romeo takes " + choreo.getTrajectoryDuration()
                        + " seconds to perform fully.");
        return choreo;
    }

    
    public FiniteTrajectory4d getJulietTrajectory() {

    	// deal with first leg, and synchronize before corkscrew
    	double romeoDurationFirstLeg = this.firstLegRomeo.getTrajectoryDuration();
    	double julietDurationFirstLeg = this.firstLegJuliet.getTrajectoryDuration();
    	double waitToSynchroize = 0;
    	if (julietDurationFirstLeg < romeoDurationFirstLeg) {
    		waitToSynchroize = romeoDurationFirstLeg - julietDurationFirstLeg;
    	}
    	double waitBeforeCorkScrew = waitToSynchroize + waitAtStation;
    	
    	// deal with last leg
        FiniteTrajectory4d lastLeg = Trajectories
                .newStraightLineTrajectory(this.wp2Juliet, this.finalPosJuliet, exitVelocity);

        // corkscrew
        double height = 2;
        Point3D dest = Point3D.plus(circleCenterPoint, Point3D.create(0, 0, height));
        double vertSpeed = height / (circleTiming / 2);
        double updatedPhase = phaseToConnectStartJuliet + (revolutions * Math.PI); // 2PI * revs/2

        FiniteTrajectory4d corkscrewUp = Trajectories.corkscrewTrajectoryBuilder()
                .setOrigin(Point4D.from(circleCenterPoint, orientation)).setDestination(dest)
                .setSpeed(vertSpeed).setFrequency(-frequency)
                .setPhase(phaseToConnectStartJuliet).setRadius(circleRadius).build();
        FiniteTrajectory4d corkscrewDown = Trajectories.corkscrewTrajectoryBuilder()
                .setOrigin(Point4D.from(dest, orientation)).setDestination(circleCenterPoint)
                .setSpeed(vertSpeed).setFrequency(frequency)
                .setPhase(updatedPhase).setRadius(circleRadius).build();

        // constract trajectory
        TrajectoryComposite choreo = TrajectoryComposite.builder()
                .addTrajectory(Trajectories.newHoldPositionTrajectory(this.initPosJuliet))
                .withDuration(startIntroAt)
                .addTrajectory(this.firstLegJuliet)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(wp1Juliet))
                .withDuration(waitBeforeCorkScrew)
                .addTrajectory(corkscrewUp).withDuration(circleTiming / 2)
                .addTrajectory(corkscrewDown).withDuration(circleTiming / 2)
                .addTrajectory(Trajectories.newHoldPositionTrajectory(wp2Juliet))
                .withDuration(waitAtStation).addTrajectory(lastLeg)
                .build();

        LoggerFactory.getLogger(TwinDrones.class)
                .info("Trajecory for Juliet takes " + choreo.getTrajectoryDuration()
                        + " seconds to perform fully.");
        return choreo;
    }

}
