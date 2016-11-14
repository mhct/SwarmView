package io.github.agentwise.swarmview.trajectory.control;

import java.util.ArrayList;
import java.util.List;

import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * Defines the choreography (movements) of all drones in a complete dance show
 *
 * @author Mario h.c.t.
 */
public class Choreography implements ChoreographyView {
    private int numberDrones;
    private final List<Act> acts = new ArrayList<>();

    private Choreography() {
    }

    private Choreography(int numberDrones) {
        this.numberDrones = numberDrones;
    }

    public static Choreography create(int numberDrones) {
        Choreography choreo = new Choreography(numberDrones);

        return choreo;
    }

    @Override
    public int getNumberDrones() {
        return numberDrones;
    }

    public void setNumberDrones(int numberDrones) {
        this.numberDrones = numberDrones;
    }

    @Override
    public FiniteTrajectory4d getFullTrajectory(DroneName name) {
        //iterate over all acts,
        //retrieve the trajectory per dronename

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
                        return Pose.create(tempPose.x(), -tempPose.y(), tempPose.z(), tempPose.yaw());
                    }

                    initialTimeAct = finalTimeAct;
                }

                // if could not find any trajectory, returns null
                // TODO raise RuntimeException, indicating the show does not last the ammount of timeInSeconds
                throw new RuntimeException(String.format(
                        "This trajectory only lasts for %s seconds, but was invoked with %s seconds",
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
    	for (Act act: acts) {
    		duration += act.getDuration();
    	}
    	return duration;
    }

    @Override
	public String getCurrentActName(float timeStep) {
		float accumulatedTime = 0.0f;
		for (Act act: acts) {
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
    	for (DroneName drone: DroneName.values()) {
    		trajectories.add(getFullTrajectory(drone));
    	}
    	
    	return trajectories;
    }
}
