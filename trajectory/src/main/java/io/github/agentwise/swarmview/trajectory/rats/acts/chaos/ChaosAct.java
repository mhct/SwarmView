package io.github.agentwise.swarmview.trajectory.rats.acts.chaos;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Particle;
import io.github.agentwise.swarmview.trajectory.swarmmovements.Swarm;
import io.github.agentwise.swarmview.trajectory.swarmmovements.decorators.VerticalCircleDecorator;

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
                  Point4D.create(0, 1, 3.2, YAW), 0.3, 0.19, 1.0, 2.0, 3.5, 0.19, 1.5, 0.19, 5);
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(5, 1, 3.2, YAW), 0.3, 0.19, 1.0, 2.0, 3.5, 0.19, 1.3, 0.19, 10);
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(5, 4, 2.0, YAW), 0.3, 0.19, 1.0, 1.0, 2.0, 0.19, 1.3, 0.19, 10);
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(3, 4.5, 2.0, YAW), 0.3, 0.19, 1.0, 1.0, 2.0, 0.19, 1.3, 0.19, 10);
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(2, 3, 2.0, YAW), 0.3, 0.19, 1.0, 1.0, 2.0, 0.19, 1.3, 0.19, 5);
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(2, 1, 2.0, YAW), 0.3, 0.19, 1.0, 1.0, 2.0, 0.19, 1.3, 0.19, 5);
          drones
              .get(DroneName.Nerve)
              .moveNervouslyToPoint(
                  Point4D.create(5, 1, 2.0, YAW), 0.3, 0.19, 1.0, 2.0, 3.0, 0.19, 1.0, 0.19, 7);
          drones
              .get(DroneName.Nerve)
              .moveToPointWithVelocity(
                  Point4D.from(configuration.finalPositionConfiguration().get(DroneName.Nerve)),
                  1.0);

          drones.get(DroneName.Fievel).moveTriangleToPoint(Point4D.create(5, 1, 3, YAW), 1.0, 1.0);
          drones.get(DroneName.Fievel).moveTriangleToPoint(Point4D.create(5, 4, 3, YAW), 1.0, 0.8);
          drones.get(DroneName.Fievel).moveTriangleToPoint(Point4D.create(1, 4, 3, YAW), 1.0, 1.0);
          drones.get(DroneName.Fievel).moveTriangleToPoint(Point4D.create(1, 1, 3.5, YAW), 1.0, 0.7);
//          drones
//              .get(DroneName.Fievel)
//              .moveTriangleToPoint(Point4D.create(0.5, 1.0, 3.5, YAW), 1.0, 0.8);
          drones
              .get(DroneName.Fievel)
              .moveTriangleToPoint(Point4D.create(5, 1.0, 3.5, YAW), 1.0, 0.8);
          drones
              .get(DroneName.Fievel)
              .moveTriangleToPoint(Point4D.create(5.5, 4.5, 2.5, YAW), 1.0, 0.8);
          drones
              .get(DroneName.Fievel)
              .moveTriangleToPoint(
                  Point4D.from(configuration.finalPositionConfiguration().get(DroneName.Fievel)),
                  1.0,
                  1.0);

          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(0.5, 5, 2, YAW), 7.0);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(1, 4, 2, YAW), 7.0);
          drones.get(DroneName.Dumbo).wiggle(10, 1);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(1, 1, 2, YAW), 7.0);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(5, 1, 3, YAW), 7.0);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(5, 4, 2, YAW), 7.0);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(2.5, 2.5, 2, YAW), 7.0);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(4, 2, 2, YAW), 7.0);
//          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(2.5, 2.5, 1.0, YAW), 7.0);
          drones.get(DroneName.Dumbo).moveZigZagToPoint(Point4D.create(4.0, 4, 1.5, YAW), 7.0);
          drones
              .get(DroneName.Dumbo)
              .moveToPointWithVelocity(
                  Point4D.from(configuration.finalPositionConfiguration().get(DroneName.Dumbo)),
                  1.0);

          // TODO remove
          //          drones.values().forEach(drone -> drone.hover(1000));
        });
    final Act act = new Act(configuration);
    act.addTrajectory(DroneName.Nerve, swarm.get(DroneName.Nerve));
    act.addTrajectory(DroneName.Fievel, swarm.get(DroneName.Fievel));
    act.addTrajectory(DroneName.Dumbo, swarm.get(DroneName.Dumbo));
    act.addTrajectory(
        DroneName.Romeo,
        getRomeoTrajectory(
            configuration.initialPositionConfiguration().get(DroneName.Romeo),
            configuration.finalPositionConfiguration().get(DroneName.Romeo)));
    act.addTrajectory(
        DroneName.Juliet,
        getJulietTrajectory(
            configuration.initialPositionConfiguration().get(DroneName.Juliet),
            configuration.finalPositionConfiguration().get(DroneName.Juliet)));
    act.lockAndBuild();
    return act;
  }

  private static FiniteTrajectory4d getCommonTrajectoryForRomeoAndJuliet() {
    final Pose initialPose = Pose.create(5, 4, 1.5, -StrictMath.PI / 2);
    final Particle drone = new Particle(initialPose);
    drone.moveToPoint(Point4D.create(3.0, 3.0, 2.5, YAW), 5);
    drone.hover(5);
    drone.moveToPoint(Point4D.create(4.0, 4.0, 2.5, YAW), 5);
    drone.moveToPoint(Point4D.create(1.0, 4.0, 1.5, YAW), 8);
    drone.moveToPoint(Point4D.create(0.5, 4.0, 1.5, YAW), 6);
    drone.hover(2);
    drone.moveToPoint(Point4D.create(0.5, 2.0, 1.5, YAW), 5);
    drone.moveToPoint(Point4D.create(1.5, 2.0, 1.5, YAW), 3);
    return drone.getTrajectory();
  }

  private static FiniteTrajectory4d getRomeoTrajectory(Pose initialPose, Pose finalPose) {
    final FiniteTrajectory4d commonTrajectory = getCommonTrajectoryForRomeoAndJuliet();
    final FiniteTrajectory4d romeoCircleTrajectory =
        VerticalCircleDecorator.create(commonTrajectory, 0.5, 0, 0.08, Point4D.create(0, 0, 0, 0));
    final Particle romeoParticle = new Particle(initialPose);
    romeoParticle.moveToPoint(Point4D.from(romeoCircleTrajectory.getDesiredPosition(0)), 2);
    romeoParticle.addMovement(romeoCircleTrajectory);
    romeoParticle.moveToPointWithVelocity(Point4D.from(finalPose), 0.8);
    return romeoParticle.getTrajectory();
  }

  private static FiniteTrajectory4d getJulietTrajectory(Pose initialPose, Pose finalPose) {
    final FiniteTrajectory4d commonTrajectory = getCommonTrajectoryForRomeoAndJuliet();
    final FiniteTrajectory4d julietCircleTrajectory =
        VerticalCircleDecorator.create(
            commonTrajectory, 0.5, StrictMath.PI, 0.08, Point4D.create(0, -1.0, 0, 0));
    final Particle julietParticle = new Particle(initialPose);
    julietParticle.moveToPoint(Point4D.from(julietCircleTrajectory.getDesiredPosition(0)), 2);
    julietParticle.addMovement(julietCircleTrajectory);
    julietParticle.moveToPointWithVelocity(Point4D.from(finalPose), 0.8);
    return julietParticle.getTrajectory();
  }
}
