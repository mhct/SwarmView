/**
 * 
 */
package rats.acts.attack;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

import java.util.HashMap;
import java.util.LinkedHashMap;

import applications.trajectory.Trajectory4d;
import applications.trajectory.geom.point.Point3D;
import control.Act;
import control.ActConfiguration;
import control.DroneName;
import control.dto.Pose;
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
public class AttackAct2 extends Act {
	
	private AttackAct2(ActConfiguration configuration) {
		super(configuration);
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new AttackAct2 (configuration);
		
		try {
			act.addTrajectory(Nerve, AttackAct2.exampleLineTrajectory(act.initialPosition(Nerve), act.finalPosition(Nerve), act.getDuration()));
			act.addTrajectory(Romeo, AttackAct2.exampleLineTrajectory(act.initialPosition(Romeo), act.finalPosition(Romeo), act.getDuration()));
			act.addTrajectory(Juliet, AttackAct2.exampleLineTrajectory(act.initialPosition(Juliet), act.finalPosition(Juliet), act.getDuration()));
			act.addTrajectory(Fievel, AttackAct2.exampleLineTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel), act.getDuration()));
			act.addTrajectory(Dumbo, AttackAct2.exampleLineTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo), act.getDuration()));
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	
	public static Act create2 (ActConfiguration configuration) {
		
		Act act = new AttackAct2 (configuration);
		
		HashMap<DroneName, Pose> initialPositions	= new LinkedHashMap<DroneName, Pose>();
		HashMap<DroneName, Pose> finalPositions		= new LinkedHashMap<DroneName, Pose>();

		HashMap<DroneName, Pose> attackPositions;
		
		for (DroneName drone: DroneName.values()) {
			initialPositions.put( drone, act.initialPosition(drone) );
			finalPositions.put( drone, act.finalPosition(drone) );
		}
		
		AttackTrajectories firstAttack = new AttackTrajectories(							
												Point3D.create(5, 3, 0),	// position of the dancer
												2.5, 	1.5,				// height to start attack from, and height to stop
												3, 		1.5);				// radius to start attack from, and radius to stop
		AttackTrajectories secondAttack = new AttackTrajectories(							
												Point3D.create(2, 5, 0),	// position of the dancer
												2.5, 	1.5,				// height to start attack from, and height to stop
												3, 		1.5);				// radius to start attack from, and radius to stop
		GoToTrajectories moveToFirstAttackPosition = new GoToTrajectories (initialPositions, );
		GoToTrajectories moveToSecondAttackPosition = new GoToTrajectories (initialPositions, );
		GoToTrajectories moveToFinalAttackPosition = new GoToTrajectories (, finalPositions);



		for (DroneName drone: DroneName.values()) {
			
			FiniteTrajectory4d traj = new TrajectoryComposite();
			
			traj.addTrajectory(moveToFirstAttackPosition.getTrajectory(drone));
			traj.addTrajectory()
			traj.addTrajectory(firstAttack.getTrajectory(drone));
			traj.addTrajectory(moveToFirstAttachPosition.getTrajectory(drone));
			traj.addTrajectory(moveToFirstAttachPosition.getTrajectory(drone));
			act.addTrajectory(drone, traj);
			
		}

		
		int numberOfDrones = DroneName.values().length;
		act.addTrajectory(DroneName.Nerve, 	trajectories.getTrajectory (0, numberOfDrones, 0));
		act.addTrajectory(DroneName.Romeo, 	trajectories.getTrajectory (1, numberOfDrones, 1));
		act.addTrajectory(DroneName.Juliet, trajectories.getTrajectory (2, numberOfDrones, 1));
		act.addTrajectory(DroneName.Fievel, trajectories.getTrajectory (3, numberOfDrones, 2));
		act.addTrajectory(DroneName.Dumbo, 	trajectories.getTrajectory (4, numberOfDrones, 3));
		
		
		GoToTrajectories goTo = new GoToTrajectories (initialPositions, )
		/* create single attack acts */
		
		Act firstAttack = SingleAttackAct.create (
							Point3D.create(5, 3, 0),	// position of the dancer
							2.5, 	1.5,				// height to start attack from, and height to stop
							3, 		1.5);				// radius to start attack from, and radius to stop
		Act secondAttack = SingleAttackAct.create (
							Point3D.create(2, 5, 0),	// position of the dancer
							2.5,	1.5,				// height to start attack from, and height to stop
							3, 		1.5);				// radius to start attack from, and radius to stop
		
		
		/* create two acts to "move into position", i.e. one at the beginning, another one between the 2 attacks */

		attackPositions = firstAttack.getInitialPositions();
		
		Act moveToFirstAttachPosition = InterAct.create(initialPositions, attackPositions);
		
		initialPositions = firstAttack.getFinalPositions();
		attackPositions = secondAttack.getInitialPositions();
		
		Act moveToSecondAttackPosition = InterAct.create(attackPositions, startPositions);


		/* now create trajectories based on these "sub-acts" */
		
		for (DroneName drone: .act..) {
			
			FiniteTrajectory4d traj = new composite trajectory;
			traj.addTrajectory(moveToFirstAttachPosition.getTrajectory(drone));
			traj.addTrajectory(firstAttack.getTrajectory(drone));
			traj.addTrajectory(moveToFirstAttachPosition.getTrajectory(drone));
			traj.addTrajectory(moveToFirstAttachPosition.getTrajectory(drone));
			act.addTrajectory(drone, traj);
		}
		
			
		
		try {
			//TODO Parsing the trajectories configuration will be added here
			
			AttackTrajectories firstAttack = new AttackTrajectories();			// height to start attack from, and height to stop
			AttackTrajectories secondAttack = new AttackTrajectories(new Point3D(2, 5, 0),	// position of the dancer
																	2.5, 1.5);			// height to start attack from, and height to stop

			
			Trajectory4d nerve = new CompositeTrajectory();
			nerve.addTrajectory(firstAttack.getTrajectory(0, 5, 0));
			nerve.addTrajectory(secondAttack.getTrajectory(0, 5));

			Trajectory4d romeo = new CompositeTrajectory();
			romeo.addTrajectory(firstAttack.getTrajectory(1, 5, 1));
			romeo.addTrajectory(secondAttack.getTrajectory(1, 5, 1));

			Trajectory4d juliet = new CompositeTrajectory();
			juliet.addTrajectory(firstAttack.getTrajectory(2, 5, 1));
			juliet.addTrajectory(secondAttack.getTrajectory(2, 5, 1));

			Trajectory4d fievel = new CompositeTrajectory();
			fievel.addTrajectory(firstAttack.getTrajectory(3, 5, 2));
			fievel.addTrajectory(secondAttack.getTrajectory(3, 5, 2));

			
			act.addTrajectory(DroneName.Nerve, nerve);
			act.addTrajectory(DroneName.Romeo, romeo);
			act.addTrajectory(DroneName.Juliet, juliet);
			act.addTrajectory(DroneName.Fievel, fievel);
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}

}
