/**
 * 
 */
package rats.acts.interact;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import applications.trajectory.LineTrajectory;
import applications.trajectory.geom.point.Point3D;
import control.Act;
import control.ActConfiguration;
import control.DroneName;
import control.DronePositionConfiguration;
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

	private InterAct (ActConfiguration configuration) {
		super(configuration);
	}
	
	public static Act create (HashMap<DroneName, Pose> initialPoses, HashMap<DroneName, Pose> finalPoses) {

		
		List<DronePositionConfiguration> positions = new ArrayList<>();
		positions.add(DronePositionConfiguration.create(Nerve,  initialPoses.get(DroneName.Nerve),  finalPoses.get(DroneName.Nerve)));
		positions.add(DronePositionConfiguration.create(Romeo,  chaos.finalPosition(Romeo),  Pose.create(3.5, 3.0, 2.5, 0.0)));
		positions.add(DronePositionConfiguration.create(Juliet, chaos.finalPosition(Juliet), Pose.create(2.0, 6.0, 2.0, 0.0)));
		positions.add(DronePositionConfiguration.create(Fievel, chaos.finalPosition(Fievel), Pose.create(5.0, 5.5, 2.5, 0.0)));
		positions.add(DronePositionConfiguration.create(Dumbo,  chaos.finalPosition(Dumbo),  Pose.create(3.0, 6.1, 1.0, 0.0)));
		ActConfiguration attackConfiguration = ActConfiguration.create(5, positions);
		
		
		Act act = new InterAct(null);
		
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
