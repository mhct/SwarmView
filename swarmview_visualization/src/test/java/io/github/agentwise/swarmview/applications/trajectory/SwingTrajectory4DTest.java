package io.github.agentwise.swarmview.applications.trajectory;

import com.google.common.collect.Lists;

import io.github.agentwise.swarmview.applications.trajectory.Trajectories;
import io.github.agentwise.swarmview.applications.trajectory.Trajectory4d;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;

import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static io.github.agentwise.swarmview.applications.trajectory.TestUtils.assertBounds;
import static junit.framework.TestCase.assertEquals;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public class SwingTrajectory4DTest {

    private Trajectory4d target;
    private Trajectory4d targetPlaneShift;
    private Point4D origin = Point4D.create(2, 10, 5, 0);
    private double radius = 1d;
    private double frequency = 0.1;
    private double planeshift = Math.PI / 2d;

    @Before
    public void setUp() {
        target = Trajectories.newPendulumSwingTrajectory(origin, radius, frequency, 0);
        targetPlaneShift =
                Trajectories.newPendulumSwingTrajectory(origin, radius, frequency, planeshift);
    }

    @Test
    public void getTrajectoryLinearXTestBounds() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(target.getDesiredPositionX(i / 10d));
        }
        assertBounds(l, origin.getX() - radius, origin.getX() + radius);
    }

    @Test
    public void getTrajectoryLinearYTestBounds() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(target.getDesiredPositionY(i / 10d));
        }
        assertBounds(l, origin.getY(), origin.getY());
    }

    @Test
    public void getTrajectoryLinearZTestBounds() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(target.getDesiredPositionZ(i / 10d));
        }
        assertBounds(l, origin.getZ() - radius, origin.getZ());
    }

    @Test
    public void getTrajectoryAngularZTestBounds() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(target.getDesiredPositionZ(i / 10d));
        }
        assertBounds(l, 0, Math.PI * 2);
    }

    @Test
    public void testYawAngleFixed() {
        double yawAngle = 0.5;
        Trajectory4d target1 =
                Trajectories.swingTrajectoryBuilder()
                        .setOrigin(Point4D.create(1.5, -2, 2.5, yawAngle))
                        .setFrequency(0.067)
                        .setRadius(1.5)
                        .build();
        assertEquals(yawAngle, target1.getDesiredAngleZ(10), 0);
    }

    @Test
    public void getTrajectoryLinearXTestBoundsPlaneShift() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(targetPlaneShift.getDesiredPositionX(i / 10d));
        }
        assertBounds(l, origin.getX(), origin.getX());
    }

    @Test
    public void getTrajectoryLinearYTestBoundsPlaneShift() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(targetPlaneShift.getDesiredPositionY(i / 10d));
        }
        assertBounds(l, origin.getY() - radius, origin.getY() + radius);
    }

    @Test
    public void getTrajectoryLinearZTestBoundsPlaneShift() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(targetPlaneShift.getDesiredPositionZ(i / 10d));
        }
        assertBounds(l, origin.getZ() - radius, origin.getZ());
    }

    @Test
    public void getTrajectoryAngularZTestBoundsPlaneShift() throws Exception {
        List<Double> l = Lists.newArrayList();
        for (int i = 0; i < 1000; i++) {
            l.add(targetPlaneShift.getDesiredPositionZ(i / 10d));
        }
        assertBounds(l, 0, Math.PI * 2);
    }
}
