package io.github.agentwise.swarmview.trajectory.rats;

import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.Choreography;
import io.github.agentwise.swarmview.trajectory.control.ChoreographyView;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.DronePositionConfiguration;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.rats.acts.attack.AttackAct;
import io.github.agentwise.swarmview.trajectory.rats.acts.chaos.ChaosAct;
import io.github.agentwise.swarmview.trajectory.rats.acts.introduction.IntroductionAct;
import io.github.agentwise.swarmview.trajectory.rats.acts.taming.TamingAct;

import java.util.ArrayList;
import java.util.List;

/**
 * Defines the whole show for rats
 *
 * @author Mario h.c.t.
 */
public class RatsShow {
  private static final double YAW =-StrictMath.PI/2;

    public static ChoreographyView createChoreography() {
        //
        //Specification of initial drone positions for Introduction
        //
        List<DronePositionConfiguration> introPositions = new ArrayList<>();
      introPositions.add(
          DronePositionConfiguration.create(
              DroneName.Nerve, Pose.create(6.7, 5.0, 1.0, YAW), Pose.create(2.0, 0.0, 3.0, YAW)));
      introPositions.add(
          DronePositionConfiguration.create(
              DroneName.Romeo, Pose.create(0.0, 5.0, 1.0, YAW), Pose.create(6.0, 5.0, 1.0, YAW)));
      introPositions.add(
          DronePositionConfiguration.create(
              DroneName.Juliet, Pose.create(0.0, 3.55, 1.0, YAW), Pose.create(6.7, 5.0, 1.0, YAW)));
      introPositions.add(
          DronePositionConfiguration.create(
              DroneName.Fievel, Pose.create(0.0, 2.0, 1.0, YAW), Pose.create(0.0, 3.5, 1.5, YAW)));
      introPositions.add(
          DronePositionConfiguration.create(
              DroneName.Dumbo, Pose.create(6.7, 3.0, 1.0, YAW), Pose.create(4.0, 3.5, 2.5, YAW)));
        ActConfiguration introConfiguration = ActConfiguration
                .create("Introduction", introPositions);

        Act introduction = IntroductionAct.create(introConfiguration);
        introduction.lockAndBuild();

        //
        //Specification of initial drone positions for Chaos
        //
        List<DronePositionConfiguration> chaosPositions = new ArrayList<>();
        chaosPositions.add(DronePositionConfiguration
                .create(DroneName.Nerve, introduction.finalPosition(DroneName.Nerve),
                        Pose.create(5.7, 4.0, 2.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(DroneName.Romeo, introduction.finalPosition(DroneName.Romeo),
                        Pose.create(3.5, 2.0, 1.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(DroneName.Juliet, introduction.finalPosition(DroneName.Juliet),
                        Pose.create(1.0, 0.0, 2.5, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(DroneName.Fievel, introduction.finalPosition(DroneName.Fievel),
                        Pose.create(2.0, 4.0, 2.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(DroneName.Dumbo, introduction.finalPosition(DroneName.Dumbo),
                        Pose.create(1.5, 3.0, 1.0, 0.0)));
        ActConfiguration chaosConfiguration = ActConfiguration.create("Chaos", chaosPositions);
        Act chaos = ChaosAct.create(chaosConfiguration);
        chaos.lockAndBuild();

        //
        //Specification of initial drone positions for Attack
        //
        List<DronePositionConfiguration> attackPositions = new ArrayList<>();
        attackPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Nerve, chaos.finalPosition(DroneName.Nerve),
                        Pose.create(4.5, 1.0, 2.0, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Romeo, chaos.finalPosition(DroneName.Romeo),
                        Pose.create(3.5, 1.0, 2.5, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Juliet, chaos.finalPosition(DroneName.Juliet),
                        Pose.create(2.0, 3.55, 2.0, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Fievel, chaos.finalPosition(DroneName.Fievel),
                        Pose.create(5.0, 4, 2.5, 0.0)));
        attackPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Dumbo, chaos.finalPosition(DroneName.Dumbo),
                        Pose.create(3.0, 4.1, 1.0, 0.0)));
        ActConfiguration attackConfiguration = ActConfiguration.create("Attack", attackPositions);
        Act attack = AttackAct.create(attackConfiguration);
        attack.lockAndBuild();

        //
        //Specification of initial drone positions for Taming
        //
        List<DronePositionConfiguration> tamingPositions = new ArrayList<>();
        tamingPositions.add(DronePositionConfiguration
                .create(DroneName.Nerve, attack.finalPosition(DroneName.Nerve),
                        Pose.create(1.36, 1.0, 3.5, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(DroneName.Romeo, attack.finalPosition(DroneName.Romeo),
                        Pose.create(2.42, 2.0, 2.9, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Juliet, attack.finalPosition(DroneName.Juliet),
                        Pose.create(3.48, 3.0, 2.3, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(
                        DroneName.Fievel, attack.finalPosition(DroneName.Fievel),
                        Pose.create(4.54, 4.0, 1.7, 0.0)));
        tamingPositions.add(DronePositionConfiguration
                .create(DroneName.Dumbo, attack.finalPosition(DroneName.Dumbo),
                        Pose.create(5.6, 5.0, 1.0, 0.0)));
        ActConfiguration tamingConfiguration = ActConfiguration.create("Taming", tamingPositions);
        Act taming = TamingAct.create(tamingConfiguration);
        taming.lockAndBuild();

        //
        // Configures the whole TrajectoryComposite
        //
        final Choreography choreo = Choreography.create(
        		DroneName.Nerve,
        		DroneName.Romeo,
        		DroneName.Juliet,
        		DroneName.Fievel,
        		DroneName.Dumbo);

//        choreo.addAct(introduction);
//        choreo.addAct(chaos);
        choreo.addAct(attack);
//        choreo.addAct(taming);

        return choreo;
    }
}
