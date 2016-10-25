/**
 * 
 */
package rats.acts.attack;

import java.util.HashMap;
import java.util.LinkedHashMap;

import applications.trajectory.Trajectory4d;
import applications.trajectory.geom.point.Point3D;
import control.Act;
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
public class AttackAct extends Act {
	
	private AttackAct() { }
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create() {
		
		Act act = new AttackAct ();
		
		HashMap<DroneName, Pose> initialPositions = new LinkedHashMap<DroneName, Pose>();
		HashMap<DroneName, Pose> attackPositions;
		
		initialPositions.put( DroneName.Nerve,	Point3D.create(2,  3,  2) );
		initialPositions.put( DroneName.Romeo,	Point3D.create(4,  3,  2) );
		initialPositions.put( DroneName.Juliet, Point3D.create(2,  4,  2) );
		initialPositions.put( DroneName.Fievel, Point3D.create(2,  6,  2) );
		initialPositions.put( DroneName.Dumbo,	Point3D.create(3,  1,  2) );
				
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
