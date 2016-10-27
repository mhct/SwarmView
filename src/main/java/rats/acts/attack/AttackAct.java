package rats.acts.attack;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

import applications.trajectory.StraightLineTrajectory4D;
import applications.trajectory.geom.point.Point4D;
import control.Act;
import control.ActConfiguration;
import control.FiniteTrajectory4d;
import control.dto.Pose;

public class AttackAct extends Act {

	private AttackAct(ActConfiguration configuration) {
		super(configuration);
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new AttackAct(configuration);
		
		try {
			act.addTrajectory(Nerve, AttackAct.exampleLineTrajectory(act.initialPosition(Nerve), act.finalPosition(Nerve), 20));
			act.addTrajectory(Romeo, AttackAct.exampleLineTrajectory(act.initialPosition(Romeo), act.finalPosition(Romeo), 20));
			act.addTrajectory(Juliet, AttackAct.exampleLineTrajectory(act.initialPosition(Juliet), act.finalPosition(Juliet), 30));
			act.addTrajectory(Fievel, AttackAct.exampleLineTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel), 30));
			act.addTrajectory(Dumbo, AttackAct.exampleLineTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo), 30));
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	
	private static FiniteTrajectory4d exampleLineTrajectory(Pose initialPosition, Pose finalPosition, double duration) {
		return StraightLineTrajectory4D.createWithCustomTravelDuration(Point4D.from(initialPosition), Point4D.from(finalPosition), duration);
	}
}
