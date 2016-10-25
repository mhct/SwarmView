/**
 * 
 */
package rats.acts.attack;

import applications.trajectory.geom.point.Point3D;
import control.Act;
import control.DroneName;

/**
 * @author tom
 *
 */
public class SingleAttackAct extends Act {
	
	private SingleAttackAct () { }
	
	public static Act create (Point3D point3d, double standBackHeight, double attackHeight, double standBackRadius, double attackRadius) {
		
		Act act = new SingleAttackAct ();
		
		AttackTrajectories trajectories = new AttackTrajectories(point3d, standBackHeight, attackHeight, standBackRadius, attackRadius);
		int numberOfDrones = DroneName.values().length;
		act.addTrajectory(DroneName.Nerve, 	trajectories.getTrajectory (0, numberOfDrones, 0));
		act.addTrajectory(DroneName.Romeo, 	trajectories.getTrajectory (1, numberOfDrones, 1));
		act.addTrajectory(DroneName.Juliet, trajectories.getTrajectory (2, numberOfDrones, 1));
		act.addTrajectory(DroneName.Fievel, trajectories.getTrajectory (3, numberOfDrones, 2));
		act.addTrajectory(DroneName.Dumbo, 	trajectories.getTrajectory (4, numberOfDrones, 3));
		
		return act;
		
	}

}
