package io.github.agentwise.swarmview.trajectory.rats.acts.chaos;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;

public class ChaosAct extends Act {
  protected static final double YAW = -Math.PI / 2;

  private ChaosAct(ActConfiguration configuration) {
    super(configuration);
  }

  /**
   * Adds all the movements of this act
   *
   * @return
   */
  public static Act create(ActConfiguration configuration) {
    final Swarm swarm = Swarm.create(configuration.initialPositionConfiguration());
    swarm.setSwarmMovementsScript(
        drones -> {
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(0, 0, 3.2, YAW), 0.3, 0.19, 1.0, 1.0, 3.5, 0.19, 1.5, 0.19, 10);

          drones.get(DroneName.Fievel).moveTriangleToPoint(Point4D.create(6, 0, 3, YAW), 1.0, 1.5);

          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(0, 4, 2, YAW), 5.0);

          drones.values().forEach(drone -> drone.hover(1000));
        });
    return Act.createWithSwarm(configuration, swarm);
  }
}
