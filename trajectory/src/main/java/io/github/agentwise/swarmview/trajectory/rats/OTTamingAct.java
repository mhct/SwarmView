package io.github.agentwise.swarmview.trajectory.rats;

import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.Choreography;
import io.github.agentwise.swarmview.trajectory.control.ChoreographyView;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.DronePositionConfiguration;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.rats.acts.taming.TamingAct;

import java.util.ArrayList;
import java.util.List;

/** @author Hoang Tung Dinh */
public final class OTTamingAct {
  public static final double YAW = -Math.PI / 2.0;
  private static ActConfiguration config;

  static {
    List<DronePositionConfiguration> tamingPositions = new ArrayList<>();
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Nerve, Pose.create(5, 3, 1, YAW), Pose.create(1.36, 1.0, 3.5, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Romeo, Pose.create(3.5, 1.0, 2.5, YAW), Pose.create(2.42, 2.0, 2.9, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Juliet, Pose.create(2.0, 0, 1.0, YAW), Pose.create(3.48, 3.0, 2.3, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Fievel, Pose.create(1,3,1, YAW), Pose.create(4.54, 4.0, 1.7, YAW)));
    tamingPositions.add(
        DronePositionConfiguration.create(
            DroneName.Dumbo, Pose.create(3.0, 4.1, 1.0, YAW), Pose.create(5.6, 5.0, 1.0, YAW)));

    config = ActConfiguration.create("TammingAct", tamingPositions);
  }

  public static ChoreographyView createTamingChoreography() {
    final Act tamingAct = TamingAct.create(config);
    tamingAct.lockAndBuild();

    final Choreography choreo =
        Choreography.create(
            DroneName.Nerve, DroneName.Romeo, DroneName.Juliet, DroneName.Fievel, DroneName.Dumbo);
    choreo.addAct(tamingAct);
    return choreo;
  }
}
