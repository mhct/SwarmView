/**
 * 
 */
package io.github.agentwise.rats.acts.attack;


import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.control.Act;
import io.github.agentwise.control.ActConfiguration;
import io.github.agentwise.control.DroneName;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;
import io.github.agentwise.rats.acts.interact.HoverAct;
import io.github.agentwise.rats.acts.interact.InterAct;

/**
 *  Attack Act definition
 *
 * @author tom
 *
 */
public class AttackAct extends Act {
	
	private static final double WAIT_BEFORE_SINGLE_ATTACK = 1;

	private AttackAct(ActConfiguration configuration) {
		super(configuration);
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		
		double[][] path = {
				{ 	5, 3, 0,		3.5, 1,		2.5, 0.75 },	// dancer position, height to start attack from, and height to stop, radius to start attack from, and radius to stop
				{ 	2, 5, 0,		2.5, 1,		1.5, 0.5  },	// dancer position, height to start attack from, and height to stop, radius to start attack from, and radius to stop
				{ 	5, 5, 0,		3.5, 1,		2.5, 0.75 },	// dancer position, height to start attack from, and height to stop, radius to start attack from, and radius to stop
		};

		Act act = new AttackAct (configuration);
		Map<DroneName, Pose> currentPositions = act.initialPositions();
		
		List<Act> acts = new ArrayList<Act>();

		for(int i = 0 ; i < path.length; i++) {
			double[] lineInfo = path[i];

			Point3D centerPoint = Point3D.create( lineInfo[0], lineInfo[1], lineInfo[2]);
			double heightToAttackFrom = lineInfo[3];
			double heightToStop = lineInfo[4];
			double radiusToAttackFrom = lineInfo[5];
			double radiusToStop = lineInfo[6];
			
			Act singleAttack = SingleAttackAct.create(	centerPoint,
														heightToAttackFrom, heightToStop,
														radiusToAttackFrom, radiusToStop);
			singleAttack.lockAndBuild();
			
			Act moveToAttackPositions = InterAct.create(currentPositions, singleAttack.initialPositions());
			moveToAttackPositions.lockAndBuild();
			
			currentPositions = singleAttack.finalPositions();
			
			Act holdBeforeAttack = HoverAct.create(currentPositions, WAIT_BEFORE_SINGLE_ATTACK);
			holdBeforeAttack.lockAndBuild();
			
			acts.add(moveToAttackPositions);
			acts.add(holdBeforeAttack);
			acts.add(singleAttack);
		}
		Act moveToFinalPositions = InterAct.create(currentPositions, act.finalPositions());
		moveToFinalPositions.lockAndBuild();
		acts.add(moveToFinalPositions);
		

		for (DroneName drone: DroneName.values()) {
			
			Builder trajectoryBuilder = TrajectoryComposite.builder();
			
			for (Act subAct: acts) {
				trajectoryBuilder.addTrajectory(subAct.getTrajectory(drone));
			}
			
			FiniteTrajectory4d droneTrajectory = trajectoryBuilder.build(); 
			act.addTrajectory(drone, droneTrajectory);
			
		}
		
		return act;
	}

}
