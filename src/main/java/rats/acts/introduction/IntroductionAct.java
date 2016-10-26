package rats.acts.introduction;

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

/**
 * Introduction Act definition
 * TODO add configuration (drones, initial pose, final pose) to the act
 * 
 * @author Mario h.c.t.
 *
 */
public class IntroductionAct extends Act {

	private IntroductionAct(ActConfiguration configuration) {
		super(configuration);
	}
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create(ActConfiguration configuration) {
		Act act = new IntroductionAct(configuration);
		
		try {
			//TODO Parsing the trajectories configuration will be added here
			act.addTrajectory(Nerve, new NerveTrajectoryIntroduction(act.initialPosition(Nerve), act.finalPosition(Nerve)));
			act.addTrajectory(Romeo, TwinDrones.createRomeoTrajectory());
			act.addTrajectory(Juliet, TwinDrones.createJulietTrajectory());
			act.addTrajectory(Fievel, IntroductionAct.exampleLineTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel), 10));
			act.addTrajectory(Dumbo, IntroductionAct.exampleLineTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo), 10));
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	
	private static FiniteTrajectory4d exampleLineTrajectory(Pose initialPosition, Pose finalPosition, double duration) {
		return StraightLineTrajectory4D.createWithCustomTravelDuration(Point4D.from(initialPosition), Point4D.from(finalPosition), duration);
	}
}
