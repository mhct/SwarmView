/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.attack;

import com.google.common.collect.Maps;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.HorizontalCircleDecorator;

import java.util.Map;

/**
 * Attack Act definition
 *
 * @author tom
 */
public class AttackAct extends Act {

  private static final double YAW = -StrictMath.PI / 2;

  private AttackAct(ActConfiguration configuration) {
    super(configuration);
  }

  /**
   * Adds all the movements of this act
   *
   * @return
   */
  public static Act create(ActConfiguration configuration) {
     final Map<DroneName, Point4D> initialAttackPosition = Maps.newHashMap();
    initialAttackPosition.put(DroneName.Juliet, Point4D.create(1, 0, 3.5, YAW));
    initialAttackPosition.put(DroneName.Romeo, Point4D.create(4, 0, 3.5, YAW));
    initialAttackPosition.put(DroneName.Nerve, Point4D.create(6, 1.5, 3.5, YAW));
    initialAttackPosition.put(DroneName.Fievel, Point4D.create(4, 3, 3.5, YAW));
    initialAttackPosition.put(DroneName.Dumbo, Point4D.create(1, 3, 3.5, YAW));
    final Point4D dancerPosition = Point4D.create(3.5, 1.5, 1.7, YAW);

    final Map<DroneName, Double> durationToReachInitialAttackPosition = Maps.newHashMap();

    final Swarm swarm = Swarm.create(configuration.initialPositionConfiguration());
    swarm.setSwarmMovementsScript(drones -> {
      drones.forEach((drone, particle) -> particle.moveToPointWithVelocity(initialAttackPosition.get(drone), 2));
      drones.forEach((drone, particle) -> durationToReachInitialAttackPosition.put(drone, particle.getTrajectory().getTrajectoryDuration()));

      for (int i = 0; i < 10; i++) {
      drones.values().forEach(particle -> particle.moveTowardPointAndStopRandomlyBeforeReachingPoint(dancerPosition, 0.5, 1.0, 1.0, 1.5));
      drones.forEach((drone, particle) -> particle.moveTowardPointAndStopRandomlyWithInRange(initialAttackPosition.get(drone), 0.5, 1.5));

      }

      drones.forEach((drone, particle) -> particle.moveToPointWithVelocity(Point4D.from(configuration.finalPositionConfiguration().get(drone)), 2));
  });


   final Act act = new Act(configuration);
   swarm.getDroneNames().forEach(drone -> act.addTrajectory(drone, HorizontalCircleDecorator.create(swarm.get(drone), Point3D.project(dancerPosition), 0.1, durationToReachInitialAttackPosition.get(drone))));
  act.lockAndBuild();
  return act;
  }
}
