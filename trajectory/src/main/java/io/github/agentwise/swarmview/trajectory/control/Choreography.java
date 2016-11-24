package io.github.agentwise.swarmview.trajectory.control;

import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

import java.util.ArrayList;
import java.util.List;

import com.google.common.collect.Lists;

import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * Defines the choreography (movements) of all drones in a complete dance show.
 * The frame of reference for a trajectory is defined as
 *
 * ----> x (positive)
 * |
 * \/ y (positive)
 *
 *
 *
 * @author Mario h.c.t.
 */
public class Choreography implements ChoreographyView {
    private final List<Act> acts = new ArrayList<>();
	final private List<DroneName> drones;

    private Choreography(DroneName...drones) {
        this.drones = Lists.newArrayList(drones);
    }

    /**
     * Creates a choreography with a fixed number of drones.
     *
     * @param numberDrones
     * @return
     */
    public static Choreography create(DroneName... drones) {
        Choreography choreo = new Choreography(drones);

        return choreo;
    }

    @Override
    public int getNumberDrones() {
        return drones.size();
    }

    /**
     * Returns a complete (full) drone trajectory for
     * all Acts defined on a choreography.
     *
     * This method return trajectories which are transformed between the Coordinate Frame of
     * the RatsView and the Coordinate Frame used in BeSwarm.
     *
     * @param DroneName Name of drone of interest
     *
     * @return FiniteTrajectory4d with the whole drone trajectory
     */
    @Override
    public FiniteTrajectory4d getFullTrajectory(DroneName name) {

        double accumulatedTime = 0;
        for (Act act : acts) {
            accumulatedTime += act.getDuration();
        }
        final double choreographyDuration = accumulatedTime;
        final DroneName droneName = name;

        return new FiniteTrajectory4d() {
            private List<Act> show = acts;

            @Override
            public double getTrajectoryDuration() {
                return choreographyDuration;
            }

            @Override
            public Pose getDesiredPosition(double timeInSeconds) {
                double initialTimeAct = 0;
                for (Act act : show) {
                    double finalTimeAct = act.getDuration() + initialTimeAct;

                    if (initialTimeAct <= timeInSeconds && timeInSeconds <= finalTimeAct) {
                        Pose tempPose = act.getTrajectory(droneName)
                                .getDesiredPosition(timeInSeconds - initialTimeAct);
                        return Pose
                                .create(tempPose.x(), -tempPose.y(), tempPose.z(), tempPose.yaw());
                    }

                    initialTimeAct = finalTimeAct;
                }

                // if could not find any trajectory, returns null
                // TODO raise RuntimeException, indicating the show does not last the ammount of
                // timeInSeconds
                throw new RuntimeException(String.format(
                        "This trajectory only lasts for %s seconds, but was invoked with %s "
                                + "seconds",
                        getTrajectoryDuration(), timeInSeconds));
            }
        }; // end FiniteTrajectory4d implementation
    }

    public void addAct(Act act) {
        acts.add(act);
    }

    @Override
    public double getChoreographyDuration() {
        double duration = 0.0;
        for (Act act : acts) {
            duration += act.getDuration();
        }
        return duration;
    }

    @Override
    public String getCurrentActName(float timeStep) {
        float accumulatedTime = 0.0f;
        for (Act act : acts) {
            if (timeStep >= accumulatedTime && timeStep < accumulatedTime + act.getDuration()) {
                return act.getActName();
            }
            accumulatedTime += act.getDuration();
        }

        return "no act";
    }

    @Override
    public List<FiniteTrajectory4d> getAllTrajectories() {
    	List<FiniteTrajectory4d> trajectories = new ArrayList<FiniteTrajectory4d>();
    	for (DroneName drone: drones) {
    		trajectories.add(getFullTrajectory(drone));
    	}

    	return trajectories;
    }

	@Override
	public List<DroneName> getDroneNames() {
		return new ArrayList<>(drones);
	}
}
