package io.github.agentwise.swarmview.applications.trajectory.composites;

import io.github.agentwise.swarmview.applications.trajectory.Trajectories;
import io.github.agentwise.swarmview.applications.trajectory.Trajectory4d;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.control.dto.Pose;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import static io.github.agentwise.swarmview.applications.trajectory.TestUtils.verifyTrajectoryPos4D;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public class TrajectoryCompositeTest {

    private TrajectoryComposite choreotarget;
    private double duration = 20d;
    private Trajectory4d pathTrajectory;
    private Trajectory4d holdTrajectory;
    private Point4D point;
    private double radius = 0.2d;
    private double frequency = 0.1d;

    @Before
    public void setUp() {
        point = Point4D.create(5, 5, 5, 2);
        holdTrajectory = Trajectories.newHoldPositionTrajectory(point);
        pathTrajectory = Trajectories.newCircleTrajectory4D(Point3D.origin(), radius, frequency, 0);
        this.choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(holdTrajectory)
                        .withDuration(duration)
                        .addTrajectory(pathTrajectory)
                        .withDuration(duration)
                        .build();
    }

    @Test
    public void testTwoSegmentTrajectoryChoreo() {
        verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
        //First invocation past duration still get's old point. all following
        // trigger change in segment for first call.
        choreotarget.getDesiredPosition(0d + duration);
        verifyTrajectoryPos4D(
                choreotarget, (1 / frequency) + duration, Point4D.create(radius, 0, 0, 0));
    }

    @Test
    public void testComplexExample() {
        FiniteTrajectory4d first =
                Trajectories.newStraightLineTrajectory(
                        Point4D.create(0, 0, 1.5, 0), Point4D.create(5, 5, 5, 0), 0.6);
        Trajectory4d second =
                Trajectories.newCircleTrajectory4D(Point3D.create(4, 5, 5), 1, 0.10, Math.PI / 4);
        Trajectory4d third = Trajectories.newHoldPositionTrajectory(Point4D.create(1, 1, 2, 0));
        TrajectoryComposite choreo =
                TrajectoryComposite.builder()
                        .addTrajectory(first)
                        .withDuration(20)
                        .addTrajectory(second)
                        .withDuration(40)
                        .addTrajectory(third)
                        .withDuration(30)
                        .build();
        Pose p = choreo.getDesiredPosition(1);
        assertNotEquals(0, p.x());
        assertNotEquals(0, p.y());
        assertNotEquals(0, p.z());
        assertEquals(0, p.yaw(), 0);
    }

    @Test
    @Ignore("This test is ignored because the specifications changed. We now assume all "
            + "trajectories start at time t=0s. This test will be removed in later versions.")
    public void testWithRealStartTimes() {
        double timeShift = 380;
        choreotarget.getDesiredPosition(timeShift);
        verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
        //First invocation past duration still get's old point. all following
        // trigger change in segment for first call.
        choreotarget.getDesiredPosition(timeShift + 0d + duration);
        verifyTrajectoryPos4D(
                choreotarget, (1 / frequency) + timeShift + duration,
                Point4D.create(radius, 0, 0, 0));
    }

    @Test
    public void testGetDuration() {
        assertEquals(2 * duration, choreotarget.getTrajectoryDuration(), 0);
    }

    @Test
    public void testUntillBuilder() {
        double untill = 50;
        holdTrajectory = Trajectories.newHoldPositionTrajectory(point);
        pathTrajectory = Trajectories.newCircleTrajectory4D(Point3D.origin(), radius, frequency, 0);
        this.choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(holdTrajectory)
                        .withDuration(duration)
                        .addTrajectory(pathTrajectory)
                        .withDuration(duration).addTrajectory(holdTrajectory).untillTotalDuration(untill)
                        .build();

        assertEquals(untill, choreotarget.getTrajectoryDuration(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testUntillBuilderIllegal() {
        double untill = 20;
        holdTrajectory = Trajectories.newHoldPositionTrajectory(point);
        pathTrajectory = Trajectories.newCircleTrajectory4D(Point3D.origin(), radius, frequency, 0);
        this.choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(holdTrajectory)
                        .withDuration(duration)
                        .addTrajectory(pathTrajectory)
                        .withDuration(duration).addTrajectory(holdTrajectory).untillTotalDuration(untill)
                        .build();

        assertEquals(untill, choreotarget.getTrajectoryDuration(), 0);
    }

    @Test
    public void testFiniteTrajectory() {
        choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(
                                Trajectories.newStraightLineTrajectory(
                                        Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
                        .build();
        assertEquals(2, choreotarget.getTrajectoryDuration(), 0);

        choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(
                                Trajectories.newStraightLineTrajectory(
                                        Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
                        .withDuration(5)
                        .build();
        assertEquals(5, choreotarget.getTrajectoryDuration(), 0);
        choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(
                                Trajectories.newStraightLineTrajectory(
                                        Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
                        .addTrajectory(Trajectories.newExamplePendulumSwingTrajectory())
                        .withDuration(5)
                        .build();
        assertEquals(7, choreotarget.getTrajectoryDuration(), 0);

        choreotarget =
                TrajectoryComposite.builder()
                        .addTrajectory(
                                Trajectories.newStraightLineTrajectory(
                                        Point4D.origin(), Point4D.create(1, 0, 0, 0), 0.5))
                        .addTrajectory(Trajectories.newExamplePendulumSwingTrajectory())
                        .withDuration(2)
                        .addTrajectory(Trajectories.newExampleCircleTrajectory4D())
                        .withDuration(10)
                        .build();
        assertEquals(14, choreotarget.getTrajectoryDuration(), 0);
    }

    @Test
    public void testNoMonotonicity() {
        verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
        verifyTrajectoryPos4D(
                choreotarget, (1 / frequency) + duration, Point4D.create(radius, 0, 0, 0));
        verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
        verifyTrajectoryPos4D(
                choreotarget, (1 / frequency) + duration, Point4D.create(radius, 0, 0, 0));
        verifyTrajectoryPos4D(choreotarget, 2, Point4D.create(5, 5, 5, 2));
    }
}
