package io.github.agentwise.swarmview.trajectory.applications.trajectory;

import com.google.common.collect.Lists;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;

import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertFalse;

/** @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be) */
public class CollisionDetectorTest {

  private FiniteTrajectory4d holdPos;
  private FiniteTrajectory4d lin1;
  private FiniteTrajectory4d circ1;
  private FiniteTrajectory4d circ1_phase;
  private FiniteTrajectory4d circ2;
  private FiniteTrajectory4d cork1;
  private FiniteTrajectory4d cork1_phase;
  private double EPS = 0.01;

  @Before
  public void setUp() {
    this.holdPos =
        TrajectoryComposite.builder()
            .addTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(5, 0, 5, 0)))
            .withDuration(10)
            .build();
    this.lin1 =
        Trajectories.newStraightLineTrajectory(
            Point4D.create(0, 0, 5, 0), Point4D.create(5, 0, 5, 0), 0.5);
    double freq = 0.1;
    this.circ1 =
        TrajectoryComposite.builder()
            .addTrajectory(
                Trajectories.circleTrajectoryBuilder()
                    .setRadius(1)
                    .setLocation(Point3D.create(1, 1, 5))
                    .setFrequency(freq)
                    .build())
            .withDuration(100)
            .build();
    this.circ1_phase =
        TrajectoryComposite.builder()
            .addTrajectory(
                Trajectories.circleTrajectoryBuilder()
                    .setRadius(1)
                    .setLocation(Point3D.create(1, 1, 5))
                    .setFrequency(freq)
                    .setPhase(Math.PI)
                    .build())
            .withDuration(100)
            .build();
    this.circ2 =
        TrajectoryComposite.builder()
            .addTrajectory(
                Trajectories.circleTrajectoryBuilder()
                    .setRadius(1)
                    .setLocation(Point3D.create(1, 1, 6))
                    .setFrequency(freq)
                    .build())
            .withDuration(100)
            .build();

    FiniteTrajectory4d cork =
        Trajectories.newCorkscrewTrajectory(
            Point4D.create(1, 1, 1, 0), Point3D.create(10, 10, 10), 0.5, 1, 0.10, 0);
    this.cork1 =
        TrajectoryComposite.builder()
            .addTrajectory(cork)
            .withDuration(cork.getTrajectoryDuration() - EPS)
            .build();

    cork =
        Trajectories.newCorkscrewTrajectory(
            Point4D.create(1, 1, 1, 0), Point3D.create(10, 10, 10), 0.5, 1, 0.10, Math.PI);
    this.cork1_phase =
        TrajectoryComposite.builder()
            .addTrajectory(cork)
            .withDuration(cork.getTrajectoryDuration() - EPS)
            .build();
  }

  @Test
  public void testCollisionDetectAtPoint() {
    List<CollisionDetector.Collision> collisions =
        new CollisionDetector(Lists.newArrayList(holdPos, lin1), 1).findCollisions();
    assertTrue(isCollisionPresentFor(9.9, collisions));
    assertTrue(isCollisionPresentFor(8.5, collisions));
    assertTrue(isCollisionPresentFor(8.1, collisions));
    assertFalse(isCollisionPresentFor(8.0, collisions));
    assertFalse(isCollisionPresentFor(7.9, collisions));
    assertFalse(isCollisionPresentFor(7.8, collisions));
  }

  private boolean isCollisionPresentFor(double timeT, List<CollisionDetector.Collision> coll) {
    for (CollisionDetector.Collision c : coll) {
      if (Math.abs(c.getTimePoint() - timeT) < TestUtils.EPSILON) {
        return true;
      }
    }
    return false;
  }

  @Test
  public void testNoCollisionDetectedWPhaseShift() {
    List<CollisionDetector.Collision> collisions =
        new CollisionDetector(Lists.newArrayList(circ1, circ1_phase), 1).findCollisions();
    assertTrue(collisions.isEmpty());
  }

  @Test
  public void testNoCollisionDetectedWCoordShift() {
    List<CollisionDetector.Collision> collisions =
        new CollisionDetector(Lists.newArrayList(circ1, circ2), 1).findCollisions();
    assertTrue(collisions.isEmpty());
  }

  @Test
  public void testCorkscrew() {
    List<CollisionDetector.Collision> collisions =
        new CollisionDetector(Lists.newArrayList(cork1, cork1_phase), 1).findCollisions();
    assertTrue("Found " + collisions.size() + " collisions: " + collisions, collisions.isEmpty());
  }

  @Test
  public void testNonConnectingSegments() {

    TrajectoryComposite.BuildableStepBuilder t1Builder =
        TrajectoryComposite.builder()
            .addTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(1, 0, 0, 0)))
            .withDuration(5)
            .addTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(-1, 0, 0, 0)))
            .withDuration(5);

    FiniteTrajectory4d t1 = t1Builder.build();

    TrajectoryComposite.BuildableStepBuilder t2Builder =
        TrajectoryComposite.builder()
            .addTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(-1, 0, 0, 0)))
            .withDuration(5)
            .addTrajectory(Trajectories.newHoldPositionTrajectory(Point4D.create(1, 0, 0, 0)))
            .withDuration(5);

    FiniteTrajectory4d t2 = t2Builder.build();

    List<CollisionDetector.Collision> collisions =
        new CollisionDetector(Lists.newArrayList(t1, t2), 1).findCollisions();
    assertFalse("Found " + collisions.size() + " collisions: " + collisions, collisions.isEmpty());
  }
}
