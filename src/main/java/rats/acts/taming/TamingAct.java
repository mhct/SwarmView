package rats.acts.taming;

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
		
		try {
			act.addTrajectory(Nerve, TamingAct.exampleLineTrajectory(act.initialPosition(Nerve), act.finalPosition(Nerve), 10));
			act.addTrajectory(Romeo, TamingAct.exampleLineTrajectory(act.initialPosition(Romeo), act.finalPosition(Romeo), 10));
			act.addTrajectory(Juliet, TamingAct.exampleLineTrajectory(act.initialPosition(Juliet), act.finalPosition(Juliet), 10));
			act.addTrajectory(Fievel, TamingAct.exampleLineTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel), 10));
			act.addTrajectory(Dumbo, TamingAct.exampleLineTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo), 20));
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	
	private static FiniteTrajectory4d exampleLineTrajectory(Pose initialPosition, Pose finalPosition, double duration) {
		return StraightLineTrajectory4D.createWithCustomTravelDuration(Point4D.from(initialPosition), Point4D.from(finalPosition), duration);
	}
}
