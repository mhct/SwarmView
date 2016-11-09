package io.github.agentwise.applications.trajectory;

import com.google.common.collect.Lists;

import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;

import java.util.List;

import static com.google.common.base.Preconditions.checkArgument;
import static io.github.agentwise.applications.trajectory.TrajectoryUtils.sampleTrajectory;

/**
 * A straight line trajectory in xy plane with sudden drops in the z dimension. Source and
 * destination point shoudl be in the same z plane.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public class ZDropLineTrajectory extends BasicTrajectory implements FiniteTrajectory4d {

    private final FiniteTrajectory4d target;
    private final Point4D src;
    private final Point4D dst;
    private final double velocity;

    ZDropLineTrajectory(
            Point4D before, Point4D after, double speed, double drops, double dropDistance) {
        checkArgument(
                before.getZ() == after.getZ(),
                "Origin and destination should be in the same horizontal plane.");
        this.src = before;
        this.dst = after;
        this.velocity = speed;
        TrajectoryComposite.Builder builder = TrajectoryComposite.builder();

        List<Point4D> points = Lists.newArrayList();
        List<Point4D> dropPoints = Lists.newArrayList();
        FiniteTrajectory4d traj = Trajectories.newStraightLineTrajectory(before, after, 1);
        double duration = traj.getTrajectoryDuration();
        initTraj(traj);
        for (int i = 0; i < drops; i++) {
            double mark = (duration / drops) * i;
            points.add(sampleTrajectory(traj, mark));
        }
        points.add(after);
        for (Point4D p : points) {
            dropPoints
                    .add(Point4D.create(p.getX(), p.getY(), p.getZ() - dropDistance, p.getAngle()));
        }
        for (int i = 0; i < drops; i++) {
            builder.addTrajectory(
                    Trajectories.newStraightLineTrajectory(points.get(i), dropPoints.get(i), 1));
            builder.addTrajectory(
                    Trajectories
                            .newStraightLineTrajectory(dropPoints.get(i), points.get(i + 1), 1));
        }
        target = builder.build();
    }

    private static final void initTraj(FiniteTrajectory4d traj) {
        traj.getDesiredPosition(0);
    }

    @Override
    public Pose getDesiredPosition(double timeInSeconds) {
        return Pose.create(getTargetTrajectory().getDesiredPosition(timeInSeconds));
    }

    private FiniteTrajectory4d getTargetTrajectory() {
        return target;
    }

    @Override
    public String toString() {
        return "ZDropLineTrajectory{"
                + "enterVelocity="
                + velocity
                + ", src point="
                + src
                + ", target point="
                + dst
                + '}';
    }

    @Override
    public double getTrajectoryDuration() {
        return getTargetTrajectory().getTrajectoryDuration();
    }
}
