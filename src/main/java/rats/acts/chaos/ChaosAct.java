package rats.acts.chaos;

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

public class ChaosAct extends Act {

	private ChaosAct(ActConfiguration configuration) {
		super(configuration);
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new ChaosAct(configuration);
		
		try {
			act.addTrajectory(Nerve, ChaosAct.exampleLineTrajectory(act.initialPosition(Nerve), act.finalPosition(Nerve), act.getDuration()));
			act.addTrajectory(Romeo, ChaosAct.exampleLineTrajectory(act.initialPosition(Romeo), act.finalPosition(Romeo), act.getDuration()));
			act.addTrajectory(Juliet, ChaosAct.exampleLineTrajectory(act.initialPosition(Juliet), act.finalPosition(Juliet), act.getDuration()));
			act.addTrajectory(Fievel, ChaosAct.exampleLineTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel), act.getDuration()));
			act.addTrajectory(Dumbo, ChaosAct.exampleLineTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo), act.getDuration()));
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	
	private static FiniteTrajectory4d exampleLineTrajectory(Pose initialPosition, Pose finalPosition, double duration) {
		return StraightLineTrajectory4D.createWithCustomTravelDuration(Point4D.from(initialPosition), Point4D.from(finalPosition), duration);
	}
}
