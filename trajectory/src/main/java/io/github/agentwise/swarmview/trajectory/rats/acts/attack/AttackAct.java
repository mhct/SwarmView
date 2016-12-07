/** */
package io.github.agentwise.swarmview.trajectory.rats.acts.attack;

import java.util.Map;

import com.google.common.collect.Maps;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.HorizontalCircleDecorator;

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
    initialAttackPosition.put(DroneName.Juliet, Point4D.create(2, 1.5, 3.0, YAW));
    initialAttackPosition.put(DroneName.Romeo, Point4D.create(3.5, 0, 3.0, YAW));
    initialAttackPosition.put(DroneName.Nerve, Point4D.create(5, 1.5, 3.0, YAW));
    initialAttackPosition.put(DroneName.Fievel, Point4D.create(3.5, 3, 3.0, YAW));
    initialAttackPosition.put(DroneName.Dumbo, Point4D.create(2.45, 2.55, 3.0, YAW));
    final Point4D dancerPosition = Point4D.create(3.5, 1.5, 1.7, YAW);

    final Swarm swarmMoveToAttackPosition = Swarm.create(configuration.initialPositionConfiguration());
    swarmMoveToAttackPosition.setSwarmMovementsScript(
        drones ->
            drones.forEach(
                (drone, particle) ->
                    particle.moveToPointWithVelocity(initialAttackPosition.get(drone), 2)));

    final Swarm swarmRawAttackMove = Swarm.create(swarmMoveToAttackPosition.getFinalPoses());
    swarmRawAttackMove.setSwarmMovementsScript(
        drones -> {
          for (int i = 0; i < 10; i++) {
            drones
                .values()
                .forEach(
                    particle ->
                        particle.moveTowardPointAndStopRandomlyBeforeReachingPoint(
                            dancerPosition, 0.5, 1.0, 1.5, 2.0));
            drones.forEach(
                (drone, particle) ->
                    particle.moveTowardPointAndStopRandomlyWithInRange(
                        initialAttackPosition.get(drone), 0.5, 1.5));
          }
        });

    final Swarm swarmCircleAttackMove = Swarm.create(swarmMoveToAttackPosition.getFinalPoses());
    swarmCircleAttackMove.setSwarmMovementsScript(
        drones ->
            drones.forEach(
                (drone, particle) ->
                    particle.addMovement(
                        HorizontalCircleDecorator.create(
                            swarmRawAttackMove.get(drone), Point3D.project(dancerPosition), 0.05))));

    final Swarm swarmMoveToFinalPosition = Swarm.create(swarmCircleAttackMove.getFinalPoses());
    swarmMoveToFinalPosition.setSwarmMovementsScript(
        drones ->
            drones.forEach(
                (drone, particle) ->
                    particle.moveToPointWithVelocity(
                        Point4D.from(configuration.finalPositionConfiguration().get(drone)), 2)));

    final Swarm swarmEntireMove = Swarm.create(configuration.initialPositionConfiguration());
    swarmEntireMove.setSwarmMovementsScript(
        drones ->
            drones.forEach(
                (drone, particle) -> {
                  particle.addMovement(swarmMoveToAttackPosition.get(drone));
                  particle.addMovement(swarmCircleAttackMove.get(drone));
                  particle.addMovement(swarmMoveToFinalPosition.get(drone));
                }));

    return Act.createWithSwarm(configuration, swarmEntireMove);
  }
}
