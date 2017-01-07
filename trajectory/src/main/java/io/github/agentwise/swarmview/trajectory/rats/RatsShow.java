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
  private static final double YAW = -StrictMath.PI / 2;

  private static final Act INTRODUCTION;
  private static final Act CHAOS;
  private static final Act ATTACK;
  private static final Act TAMING;

  static {
    //
    //Specification of initial drone positions for Introduction
    //
    List<DronePositionConfiguration> introPositions = new ArrayList<>();
    introPositions.add(
        DronePositionConfiguration.create(
            DroneName.Nerve, Pose.create(3.0, 3.5, 1.0, YAW), Pose.create(2.0, 1.0, 3.0, YAW)));
    introPositions.add(
        DronePositionConfiguration.create(
            DroneName.Romeo, Pose.create(0.5, 2.0, 1.0, YAW), Pose.create(5.5, 5.0, 1.0, YAW)));
    introPositions.add(
        DronePositionConfiguration.create(
            DroneName.Juliet, Pose.create(1.5, 2.0, 1.0, YAW), Pose.create(4.0, 5.0, 1.0, YAW)));
    introPositions.add(
        DronePositionConfiguration.create(
            DroneName.Fievel, Pose.create(0.5, 5.0, 1.0, YAW), Pose.create(4.5, 1.0, 2.5, YAW)));
    introPositions.add(
        DronePositionConfiguration.create(
            DroneName.Dumbo, Pose.create(5.0, 3.0, 1.0, YAW), Pose.create(4.0, 3.5, 2.5, YAW)));
    ActConfiguration introConfiguration = ActConfiguration.create("Introduction", introPositions);

    INTRODUCTION = IntroductionAct.create(introConfiguration);
    INTRODUCTION.lockAndBuild();

    //
    //Specification of initial drone positions for Chaos
    //
    List<DronePositionConfiguration> chaosPositions = new ArrayList<>();
    chaosPositions.add(
        DronePositionConfiguration.create(
            DroneName.Nerve,
            INTRODUCTION.finalPosition(DroneName.Nerve),
            Pose.create(5.0, 2.0, 2.0, YAW)));
    chaosPositions.add(
        DronePositionConfiguration.create(
            DroneName.Romeo,
            INTRODUCTION.finalPosition(DroneName.Romeo),
            Pose.create(3.5, 2.0, 1.0, YAW)));
    chaosPositions.add(
        DronePositionConfiguration.create(
            DroneName.Juliet,
            INTRODUCTION.finalPosition(DroneName.Juliet),
            Pose.create(1.0, 0.0, 2.5, YAW)));
    chaosPositions.add(
        DronePositionConfiguration.create(
            DroneName.Fievel,
            INTRODUCTION.finalPosition(DroneName.Fievel),
            Pose.create(2.0, 4.0, 2.0, YAW)));
    chaosPositions.add(
        DronePositionConfiguration.create(
            DroneName.Dumbo,
            INTRODUCTION.finalPosition(DroneName.Dumbo),
            Pose.create(4.5, 4.0, 1.0, YAW)));
    ActConfiguration chaosConfiguration = ActConfiguration.create("Chaos", chaosPositions);
    CHAOS = ChaosAct.create(chaosConfiguration);
    CHAOS.lockAndBuild();

    //
    //Specification of initial drone positions for Attack
    //
    List<DronePositionConfiguration> attackPositions = new ArrayList<>();
    attackPositions.add(
        DronePositionConfiguration.create(
            DroneName.Nerve,
            CHAOS.finalPosition(DroneName.Nerve),
            Pose.create(5.5, 3.0, 2.0, YAW)));
    attackPositions.add(
        DronePositionConfiguration.create(
            DroneName.Romeo,
            CHAOS.finalPosition(DroneName.Romeo),
            Pose.create(5.0, 1.0, 2.0, YAW)));
    attackPositions.add(
        DronePositionConfiguration.create(
            DroneName.Juliet,
            CHAOS.finalPosition(DroneName.Juliet),
            Pose.create(2.5, 1.0, 2.0, YAW)));
    attackPositions.add(
        DronePositionConfiguration.create(
            DroneName.Fievel,
            CHAOS.finalPosition(DroneName.Fievel),
            Pose.create(2.0, 3, 2.0, YAW)));
    attackPositions.add(
        DronePositionConfiguration.create(
            DroneName.Dumbo,
            CHAOS.finalPosition(DroneName.Dumbo),
            Pose.create(3.5, 4.1, 2.0, YAW)));
    ActConfiguration attackConfiguration = ActConfiguration.create("Attack", attackPositions);
    ATTACK = AttackAct.create(attackConfiguration);
    ATTACK.lockAndBuild();

    //
    //Specification of initial drone positions for Taming
    //
    List<DronePositionConfiguration> tamingPositions = new ArrayList<>();
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Nerve,
            ATTACK.finalPosition(DroneName.Nerve),
            Pose.create(6, 5, 0.8, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Romeo,
            ATTACK.finalPosition(DroneName.Romeo),
            Pose.create(3.0, 1.5, 0.8, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Fievel,
            ATTACK.finalPosition(DroneName.Fievel),
            Pose.create(0, 5, 0.8, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Juliet,
            ATTACK.finalPosition(DroneName.Juliet),
            Pose.create(5.5, 3.5, 0.8, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Dumbo,
            ATTACK.finalPosition(DroneName.Dumbo),
            Pose.create(5.0, 1.5, 0.8, YAW)));
    ActConfiguration tamingConfiguration = ActConfiguration.create("Taming", tamingPositions);
    TAMING = TamingAct.create(tamingConfiguration);
    TAMING.lockAndBuild();
  }

  public static ChoreographyView createChoreography() {
    //
    // Configures the whole TrajectoryComposite
    //
    final Choreography choreo =
        Choreography.create(
            DroneName.Nerve, DroneName.Romeo, DroneName.Juliet, DroneName.Fievel, DroneName.Dumbo);

    choreo.addAct(INTRODUCTION);
    choreo.addAct(CHAOS);
    choreo.addAct(ATTACK);
    choreo.addAct(TAMING);

    return choreo;
  }

  public static Act getIntroduction() {
    return INTRODUCTION;
  }

  public static Act getChaos() {
    return CHAOS;
  }

  public static Act getAttack() {
    return ATTACK;
  }

  public static Act getTaming() {
    return TAMING;
  }
  
}
