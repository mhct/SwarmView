/**
 * 
 */
package rats.acts.interact;

import java.util.HashMap;

import applications.trajectory.LineTrajectory;
import applications.trajectory.geom.point.Point3D;
import control.Act;
import control.DroneName;
import control.FiniteTrajectory4d;
import control.dto.Pose;
import rats.acts.attack.AttackAct;
import rats.acts.introduction.NerveTrajectoryIntroduction;

/**
 * @author tom
 *
 */
public class InterAct extends Act {
	
	private static final double TIME_BETWEEN_STARTS = 1.0;

	private InterAct() { }
	
	public static Act create (HashMap<DroneName, Pose> initialPoses, HashMap<DroneName, Pose> finalPoses) {

		Act act = new InterAct();
		
		int number = 0;
		for (DroneName drone : DroneName.values()) {
			
			Pose initPos	= initialPoses.get(drone);
			Pose finalPos 	= finalPoses.get(drone);
			
			act.addTrajectory(drone, new LineTrajectory(
											initPos,
											finalPos,
											0 + number * TIME_BETWEEN_STARTS ));
			number++;
		}
		
		return act;
	}
		
	public static Act create (Act prevAct, Act nextAct) {
		
		return create(prevAct.getFinalPoses(), nextAct.getInitialPoses());

	}
	
	

}
