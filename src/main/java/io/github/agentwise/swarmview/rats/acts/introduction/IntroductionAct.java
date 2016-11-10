package io.github.agentwise.swarmview.rats.acts.introduction;

import static io.github.agentwise.swarmview.control.DroneName.Dumbo;
import static io.github.agentwise.swarmview.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.control.DroneName.Romeo;

import io.github.agentwise.swarmview.control.Act;
import io.github.agentwise.swarmview.control.ActConfiguration;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;

/**
 * Introduction Act definition
 * TODO add configuration (drones, initial pose, final pose) to the act
 * 
 * @author Mario h.c.t.
 *
 */
public class IntroductionAct extends Act {

	private static final double TIME_BETWEEN_INTROS = 2;

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
			
			double startTime = 4;
			
			NerveTrajectoryIntroduction nerve = new NerveTrajectoryIntroduction(act.initialPosition(Nerve), act.finalPosition(Nerve), startTime);
			act.addTrajectory(Nerve, nerve);
			
			startTime = nerve.getTrajectoryDuration() + TIME_BETWEEN_INTROS;
			FiniteTrajectory4d romeo = TwinDrones.createRomeoTrajectory(act.initialPosition(Romeo), act.finalPosition(Romeo), startTime);
			act.addTrajectory(Romeo, romeo);
			FiniteTrajectory4d juliet = TwinDrones.createJulietTrajectory(act.initialPosition(Juliet), act.finalPosition(Juliet), startTime);			
			act.addTrajectory(Juliet, juliet);
			startTime = Math.max(romeo.getTrajectoryDuration(), juliet.getTrajectoryDuration()) + TIME_BETWEEN_INTROS;

			FiniteTrajectory4d fievel = FievelIntroduction.createTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel), startTime);
			act.addTrajectory(Fievel, fievel);
			startTime = fievel.getTrajectoryDuration() + TIME_BETWEEN_INTROS;

			FiniteTrajectory4d dumbo = DumboIntroduction.createTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo), startTime); 
			act.addTrajectory(Dumbo, dumbo);
			startTime = dumbo.getTrajectoryDuration() + TIME_BETWEEN_INTROS;

			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return act;
	}

}
