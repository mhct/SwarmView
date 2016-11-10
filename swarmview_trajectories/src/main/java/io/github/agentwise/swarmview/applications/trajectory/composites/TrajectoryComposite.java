package io.github.agentwise.swarmview.applications.trajectory.composites;

import com.google.auto.value.AutoValue;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import io.github.agentwise.swarmview.applications.trajectory.BasicTrajectory;
import io.github.agentwise.swarmview.applications.trajectory.Trajectory4d;
import io.github.agentwise.swarmview.applications.trajectory.TrajectoryUtils;
import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.control.dto.Pose;

import java.util.List;
import java.util.Map;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * A choreography represents a sequence of different trajectories to be executed for set durations.
 * The choreography is in itself a trajectory4d instance so it can be used as a single trajectory
 * from the lower level control point-of-view. Using the builder, one can create choreography
 * instances and configure them with different trajectories to be executed in sequence.
 * <p>A choreography can and should only be consumed once and cannot be reused.
 * Instances of TrajectoryComposite are immutable for safety purposes and should be built using
 * TrajectoryComposite.Builder().
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public final class TrajectoryComposite extends BasicTrajectory implements FiniteTrajectory4d {
    private final List<TrajectoryCompositeSegment> compositeSegments;
    private final Map<Trajectory4d, Double> absoluteStartTimes;
    private final double totalDuration;

    private TrajectoryComposite(List<TrajectoryCompositeSegment> segmentsArg) {
        super();
        compositeSegments = ImmutableList.copyOf(segmentsArg);
        absoluteStartTimes = Maps.newLinkedHashMap();
        double startTime = 0;
        for (TrajectoryCompositeSegment seg : compositeSegments) {
            this.absoluteStartTimes.put(seg.getTarget(), startTime);
            startTime += seg.getDuration();
        }
        this.totalDuration = startTime;
    }

    /**
     * @return A choreography builder instance.
     */
    public static Builder builder() {
        return new Builder();
    }

    @Override
    public Pose getDesiredPosition(double timeInSeconds) {
        Trajectory4d tSegment = getCompositeSegmentAt(timeInSeconds);
        double relativeTime = timeInSeconds - getStartTimeForSegment(tSegment);
        checkArgument(relativeTime >= 0,
                "Cannot call upon trajectory positions with negative time.");
        return Pose.create(tSegment.getDesiredPositionX(relativeTime),
                tSegment.getDesiredPositionY(relativeTime),
                tSegment.getDesiredPositionZ(relativeTime),
                tSegment.getDesiredAngleZ(relativeTime));
    }

    private Trajectory4d getCompositeSegmentAt(double timeInSeconds) {
        for (TrajectoryCompositeSegment traj : compositeSegments) {
            if (timeInSeconds >= absoluteStartTimes.get(traj.getTarget())
                    && timeInSeconds <= absoluteStartTimes.get(traj.getTarget()) + traj
                    .getDuration()) {
                return traj.getTarget();
            }
        }
        return compositeSegments.get(compositeSegments.size() - 1).getTarget();
    }

    private double getStartTimeForSegment(Trajectory4d segment) {
        return absoluteStartTimes.get(segment);
    }

    @Override
    public String toString() {
        return "TrajectoryComposite{" + "Choreo segments=" + compositeSegments + '}';
    }

    @Override
    public double getTrajectoryDuration() {
        return this.totalDuration;
    }

    /**
     * Step builder instance that can be built or further configured with trajectories.
     */
    public interface BuildableStepBuilder extends TimingRequiredStepBuilder {
        /**
         * @param trajectory The trajectory to add.
         * @return this builder instance.
         */
        BuildableStepBuilder addTrajectory(FiniteTrajectory4d trajectory);

        /**
         * @param trajectory The trajectory to add to the choreography.
         * @return A builder instance to specify the duration to execute given trajectory for.
         */
        TimingRequiredStepBuilder addTrajectory(Trajectory4d trajectory);

        /**
         * @return A fully built choreography instance.
         */
        TrajectoryComposite build();
    }

    /**
     * Step builder instance for adding timing to specified trajectory.
     */
    public interface TimingRequiredStepBuilder {
        /**
         * @param duration the duration of the previously added trajectory.
         * @return A Builder instance.
         */
        BuildableStepBuilder withDuration(double duration);

        /**
         * @param timeMark The timing mark in the future untill when to execute the current segment.
         * @return A Builder instance.
         */
        BuildableStepBuilder untillTotalDuration(double timeMark);
    }

    /**
     * A segment in the choreography specified by a target trajectory and a duration for which to
     * execute this trajectory.
     */
    @AutoValue
    abstract static class TrajectoryCompositeSegment {
        /**
         * @return The trajectory to be executed in this segment.
         */
        public abstract Trajectory4d getTarget();

        /**
         * @return The duration this trajectory should be executed for.
         */
        public abstract double getDuration();
    }

    /**
     * Builder class for building choreography instances.
     */
    public static final class Builder implements BuildableStepBuilder {

        private final List<TrajectoryCompositeSegment> segments;
        private Trajectory4d tempTarget;
        private double tempDuration;
        private boolean initial = true;

        private Builder() {
            this.segments = Lists.newArrayList();
        }

        @Override
        public TimingRequiredStepBuilder addTrajectory(Trajectory4d trajectory) {
            addSegmentWithDuration(tempTarget, tempDuration);
            this.tempTarget = trajectory;
            return this;
        }

        @Override
        public BuildableStepBuilder addTrajectory(FiniteTrajectory4d trajectory) {
            addSegmentWithDuration(tempTarget, tempDuration);
            this.tempTarget = TrajectoryUtils.createFrom(trajectory);
            this.tempDuration = trajectory.getTrajectoryDuration();
            return this;
        }

        @Override
        public TrajectoryComposite build() {
            addSegmentWithDuration(tempTarget, tempDuration);
            return new TrajectoryComposite(segments);
        }

        @Override
        public BuildableStepBuilder withDuration(double duration) {
            checkArgument(duration > 0, "Duration should be > 0");
            this.tempDuration = duration;
            return this;
        }

        @Override
        public BuildableStepBuilder untillTotalDuration(double timeMark) {
            double diffUntillMark = calcDiff(timeMark);
            if (diffUntillMark < 0) {
                throw new IllegalArgumentException(
                        "Current applications.trajectory.composites would already last longer "
                                + "than the given total duration");
            }
            return withDuration(diffUntillMark);
        }

        private double calcDiff(double timeMark) {
            return timeMark - segments.stream().mapToDouble(cS -> cS.getDuration()).sum();
        }

        private boolean hasTempState() {
            return !initial;
        }

        private void reset() {
            this.initial = false;
        }

        private void addSegmentWithDuration(Trajectory4d target, double duration) {
            if (hasTempState()) {
                segments.add(new AutoValue_TrajectoryComposite_TrajectoryCompositeSegment(target,
                        duration));
            }
            reset();
        }
    }
}
