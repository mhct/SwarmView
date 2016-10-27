/**
 * 
 */
package rats.acts.attack;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import applications.trajectory.StraightLineTrajectory4D;
import applications.trajectory.Trajectory4d;
import applications.trajectory.composites.TrajectoryComposite;
import applications.trajectory.composites.TrajectoryComposite.Builder;
import applications.trajectory.geom.point.Point3D;
import control.Act;
import control.ActConfiguration;
import control.DroneName;
import control.FiniteTrajectory4d;
import control.dto.Pose;
import rats.acts.interact.HoverAct;
import rats.acts.interact.InterAct;
import rats.acts.introduction.IntroductionAct;
import rats.acts.introduction.NerveTrajectoryIntroduction;
import rats.acts.introduction.TwinDrones;

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
		
		Act act = new AttackAct (configuration);

		Act firstAttack = SingleAttackAct.create (Point3D.create(5, 3, 0),	// position of the dancer
												3.5, 	1,					// height to start attack from, and height to stop
												2.5, 	0.75);				// radius to start attack from, and radius to stop
		firstAttack.lockAndBuild();
		
		Act secondAttack = SingleAttackAct.create (Point3D.create(2, 5, 0),	// position of the dancer
												2.5, 	1,					// height to start attack from, and height to stop
												1.5, 	0.5);				// radius to start attack from, and radius to stop
		secondAttack.lockAndBuild();


		Act moveToFirstAttackPositions = InterAct.createWithSequentialMovement(act.initialPositions(), firstAttack.initialPositions());
		moveToFirstAttackPositions.lockAndBuild();
		Act holdBeforeFirstAttack = HoverAct.create(moveToFirstAttackPositions.finalPositions(), WAIT_BEFORE_SINGLE_ATTACK);
		holdBeforeFirstAttack.lockAndBuild();
		Act moveToSecondAttackPositions = InterAct.create(firstAttack.finalPositions(), secondAttack.initialPositions());
		moveToSecondAttackPositions.lockAndBuild();
		Act holdBeforeSecondAttack = HoverAct.create(moveToSecondAttackPositions.finalPositions(), WAIT_BEFORE_SINGLE_ATTACK);
		holdBeforeSecondAttack.lockAndBuild();
		Act moveToFinalPositions = InterAct.create(secondAttack.finalPositions(), act.finalPositions());
		moveToFinalPositions.lockAndBuild();
		
		List<Act> acts = new ArrayList<Act>();
		acts.add(moveToFirstAttackPositions);
		acts.add(holdBeforeFirstAttack);
		acts.add(firstAttack);
		acts.add(moveToSecondAttackPositions);
		acts.add(holdBeforeSecondAttack);
		acts.add(secondAttack);
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
