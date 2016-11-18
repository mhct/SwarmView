package io.github.agentwise.swarmview.trajectory.rats.acts.taming;

import static io.github.agentwise.swarmview.trajectory.control.DroneName.Dumbo;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Romeo;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.rats.acts.interact.InterAct;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;

public class TamingAct extends Act {

	private TamingAct(ActConfiguration configuration) {
		super(configuration);
		
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new TamingAct(configuration);

		
		//
		// Go to initial positions
		//
		Map<DroneName, Pose> beginMovement = new LinkedHashMap<DroneName, Pose>(5);
		beginMovement.put(Nerve, Pose.create(5.0, 2.0, 1.5, 0));
		beginMovement.put(Romeo, Pose.create(3.0, 2.0, 1.5, 0));
		beginMovement.put(Fievel, Pose.create(4, 6.0, 1.5, 0));
		beginMovement.put(Dumbo, Pose.create(6.2, 4.5, 1.5, 0));
		beginMovement.put(Juliet, Pose.create(1.8, 4.5, 1.5, 0));
		
		List<DroneName> beginMovementOrder = Arrays.asList(Nerve, Romeo, Fievel, Dumbo, Juliet);
		ActConfiguration beginActConfiguration = ActConfiguration.createFromInitialFinalPositions(act.initialPositions(), beginMovement);
		Act goToPosition = InterAct.createWithOrderedSequentialMovement(beginActConfiguration, 1.0, beginMovementOrder);
		goToPosition.lockAndBuild();
		
		//
		// Perform the joint movements
		//
		Swarm swarm = Swarm.create(beginActConfiguration.finalPositionConfiguration());
		swarm.script();

		//
		// Move to final positions
		//
		Map<DroneName, Pose> moveToFinalPositionMovement = new LinkedHashMap<DroneName, Pose>(5);
		moveToFinalPositionMovement.put(Nerve, swarm.getFinalPose(Nerve));
		moveToFinalPositionMovement.put(Romeo, swarm.getFinalPose(Romeo));
		moveToFinalPositionMovement.put(Fievel, swarm.getFinalPose(Fievel));
		moveToFinalPositionMovement.put(Dumbo, swarm.getFinalPose(Dumbo));
		moveToFinalPositionMovement.put(Juliet, swarm.getFinalPose(Juliet));
		ActConfiguration goUpActConfiguration = ActConfiguration.createFromInitialFinalPositions(moveToFinalPositionMovement, act.finalPositions());
		Act goToFinalPosition = InterAct.createWithSequentialMovement(goUpActConfiguration, 1.0);
		
		try {
			for (DroneName drone: DroneName.values()) {
				act.addTrajectory(drone, TrajectoryComposite.builder()
						.addTrajectory(goToPosition.getTrajectory(drone))
						.addTrajectory(swarm.get(drone))
						.addTrajectory(goToFinalPosition.getTrajectory(drone)).build());
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
}
