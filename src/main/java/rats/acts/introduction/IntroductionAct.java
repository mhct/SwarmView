package rats.acts.introduction;

import control.Act;
import control.DroneName;

public class IntroductionAct extends Act {

	@Override
	public void initializeTrajectories() {
		try {
			addTrajectory(DroneName.Nerve, new NerveTrajectoryIntroduction());
			addTrajectory(DroneName.Romeo, TwinDrones.createRomeoTrajectory());
			addTrajectory(DroneName.Juliet, TwinDrones.createJulietTrajectory());
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
