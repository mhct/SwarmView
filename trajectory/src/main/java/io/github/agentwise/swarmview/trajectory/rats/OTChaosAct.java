package io.github.agentwise.swarmview.trajectory.rats;

import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.Choreography;
import io.github.agentwise.swarmview.trajectory.control.ChoreographyView;
import io.github.agentwise.swarmview.trajectory.control.DroneName;

/** @author Hoang Tung Dinh */
public final class OTChaosAct {
  public static ChoreographyView createChaosChoreography() {
    final Act chaosAct = RatsShow.getChaos();

    final Choreography choreo =
        Choreography.create(
            DroneName.Nerve, DroneName.Romeo, DroneName.Juliet, DroneName.Fievel, DroneName.Dumbo);
    choreo.addAct(chaosAct);
    return choreo;
  }
}
