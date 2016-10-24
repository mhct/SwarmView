package rats.acts.introduction;

import control.Act;
import control.DroneName;

/**
 * Introduction Act definition
 * TODO add configuration (drones, initial pose, final pose) to the act
 * 
 * @author Mario h.c.t.
 *
 */
public class IntroductionAct extends Act {

	private IntroductionAct() { }
	
	/**
	 * Adds all the movements of this act
	 * @return
	 */
	public static Act create() {
		Act act = new IntroductionAct();
		
		try {
			//TODO Parsing the trajectories configuration will be added here
			act.addTrajectory(DroneName.Nerve, new NerveTrajectoryIntroduction());
			act.addTrajectory(DroneName.Romeo, TwinDrones.createRomeoTrajectory());
			act.addTrajectory(DroneName.Juliet, TwinDrones.createJulietTrajectory());
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}
	

}
