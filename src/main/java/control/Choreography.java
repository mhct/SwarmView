package control;

import control.dto.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * Defines the choreography (movements) of all drones in a complete dance show
 *
 * @author Mario h.c.t.
 */
public class Choreography {
    private int numberDrones;
    private final List<Act> acts = new ArrayList<>();
    ;

    private Choreography() {
    }

    ;

    private Choreography(int numberDrones) {
        this.numberDrones = numberDrones;
    }

    public static Choreography create(int numberDrones) {
        Choreography choreo = new Choreography(numberDrones);

        return choreo;
    }

    public int getNumberDrones() {
        return numberDrones;
    }

    public void setNumberDrones(int numberDrones) {
        this.numberDrones = numberDrones;
    }

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
                        return act.getTrajectory(droneName)
                                .getDesiredPosition(timeInSeconds - initialTimeAct);
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
}
