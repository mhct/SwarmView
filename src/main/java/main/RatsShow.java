package main;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

import java.util.ArrayList;
import java.util.List;

import control.Act;
import control.ActConfiguration;
import control.Choreography;
import control.ChoreographyView;
import control.DronePositionConfiguration;
import control.dto.Pose;
import rats.acts.attack.AttackAct;
import rats.acts.chaos.ChaosAct;
import rats.acts.introduction.IntroductionAct;
import rats.acts.taming.TamingAct;

/**
 * Defines the whole show for rats
 * 
 * @author Mario h.c.t.
 *
 */
public class RatsShow {
	
	public static ChoreographyView createChoreography() {
        //
        //Specification of initial drone positions for Introduction
        //
        List<DronePositionConfiguration> introPositions = new ArrayList<>();
        introPositions.add(DronePositionConfiguration
                .create(Nerve, Pose.create(7.0, 6.0, 1.0, 0.0), Pose.create(2.0, 2.0, 4.0, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Romeo, Pose.create(7.0, 5.0, 1.0, 0.0), Pose.create(1.1, 5.0, 1.5, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Juliet, Pose.create(1.0, 5.0, 1.0, 0.0), Pose.create(6.9, 5.0, 1.5, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Fievel, Pose.create(1.0, 6.0, 1.0, 0.0), Pose.create(5.0, 2.5, 1.0, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Dumbo, Pose.create(7.0, 3.0, 1.0, 0.0), Pose.create(4.0, 3.5, 2.5, 0.0)));
        ActConfiguration introConfiguration = ActConfiguration.create("Introduction", introPositions);

        Act introduction = IntroductionAct.create(introConfiguration);
        introduction.lockAndBuild();

        //
        //Specification of initial drone positions for Chaos
        //
        List<DronePositionConfiguration> chaosPositions = new ArrayList<>();
        chaosPositions.add(DronePositionConfiguration
                .create(Nerve, introduction.finalPosition(Nerve), Pose.create(6.0, 6.0, 2.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Romeo, introduction.finalPosition(Romeo), Pose.create(3.5, 4.0, 1.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Juliet, introduction.finalPosition(Juliet),
                        Pose.create(1.0, 1.0, 2.5, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Fievel, introduction.finalPosition(Fievel),
                        Pose.create(2.0, 5.0, 2.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Dumbo, introduction.finalPosition(Dumbo), Pose.create(1.5, 3.0, 1.0, 0.0)));
        ActConfiguration chaosConfiguration = ActConfiguration.create("Chaos", chaosPositions);
        Act chaos = ChaosAct.create(chaosConfiguration);
        chaos.lockAndBuild();

        //
        //Specification of initial drone positions for Attack
        //
        List<DronePositionConfiguration> attackPositions = new ArrayList<>();
        attackPositions.add(DronePositionConfiguration
                .create(Nerve, chaos.finalPosition(Nerve), Pose.create(4.5, 3.0, 2.0, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(Romeo, chaos.finalPosition(Romeo), Pose.create(3.5, 3.0, 2.5, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(Juliet, chaos.finalPosition(Juliet), Pose.create(2.0, 6.0, 2.0, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(Fievel, chaos.finalPosition(Fievel), Pose.create(5.0, 5.5, 2.5, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(Dumbo, chaos.finalPosition(Dumbo), Pose.create(3.0, 6.1, 1.0, 0.0)));
        ActConfiguration attackConfiguration = ActConfiguration.create("Attack", attackPositions);
        Act attack = AttackAct.create(attackConfiguration);
        attack.lockAndBuild();

        //
		//Specification of initial drone positions for Taming
		//
        List<DronePositionConfiguration> tamingPositions = new ArrayList<>();
        tamingPositions.add(DronePositionConfiguration
                .create(Nerve, attack.finalPosition(Nerve), Pose.create(2.0, 2.0, 1.5, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(Romeo, attack.finalPosition(Romeo), Pose.create(3.0, 3.0, 1.5, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(Juliet, attack.finalPosition(Juliet), Pose.create(4.0, 4.0, 1.5, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(Fievel, attack.finalPosition(Fievel), Pose.create(5.0, 5.0, 1.5, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(Dumbo, attack.finalPosition(Dumbo), Pose.create(6.0, 6.1, 1.5, 0.0)));
        ActConfiguration tamingConfiguration = ActConfiguration.create("Taming", tamingPositions);
        Act taming = TamingAct.create(tamingConfiguration);
        taming.lockAndBuild();

        //
        // Configures the whole TrajectoryComposite
        //
        final Choreography choreo = Choreography.create(5);
        choreo.addAct(introduction);
        choreo.addAct(chaos);
        choreo.addAct(attack);
        choreo.addAct(taming);
		return choreo;
	}
}
