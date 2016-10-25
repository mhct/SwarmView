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
    private double durationInSec;
    private int numberDrones;
    private final List<ActConfiguration> acts = new ArrayList<>();
    ;

    private Choreography() {
    }

    ;

    private Choreography(double durationInSec, int numberDrones) {
        this.durationInSec = durationInSec;
        this.numberDrones = numberDrones;

        //ads initial Null act
        //		this.addAct(new NullAct(), 0);

        //		this.acts = new ArrayList<>();
    }

    public static Choreography create(double durationInSec, int numberDrones) {
        Choreography choreo = new Choreography(durationInSec, numberDrones);

        return choreo;
    }

    public double getDurationInSec() {
        return durationInSec;
    }

    public void setDurationInSec(double durationInSec) {
        this.durationInSec = durationInSec;
    }

    public int getNumberDrones() {
        return numberDrones;
    }

    public void setNumberDrones(int numberDrones) {
        this.numberDrones = numberDrones;
    }

    /**
     * TODO change public to private
     *
     * @return
     */
    //	public List<Act> getActs() {
    //		return new ArrayList<Act>(acts);
    //	}
    public FiniteTrajectory4d getFullTrajectory(DroneName name) {
        //iterate over all acts,
        //retrieve the trajectory per dronename

        double accumulatedTime = 0;
        for (ActConfiguration actConf : acts) {
            accumulatedTime += actConf.duration();
            //			if (Math.abs(actConf.duration() - 0.0) >= 0.0001) { //EPS to consider two
            // doubles as equal
            //			}
        }
        final double choreographyDuration = accumulatedTime;
        final DroneName droneName = name;

        return new FiniteTrajectory4d() {
            private List<ActConfiguration> show = acts;
            private int i = 0;

            @Override
            public double getTrajectoryDuration() {
                return choreographyDuration;
            }

            @Override
            public Pose getDesiredPosition(double timeInSeconds) {
                double initialTimeAct = 0;
                for (ActConfiguration actConf : show) {
                    double finalTimeAct = actConf.duration() + initialTimeAct;

                    if (initialTimeAct <= timeInSeconds && timeInSeconds <= finalTimeAct) {
                        return actConf.act().getTrajectory(droneName)
                                .getDesiredPosition(timeInSeconds - initialTimeAct);
                    }

                    initialTimeAct = finalTimeAct;
                }

                // if could not find any trajectory, returns null
                // TODO raise RuntimeException, indicating the show does not last the ammount of timeInSeconds
                // ... Seriously?
                System.out.println("NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOO\n\n\n\n");
                return null;
            }
        }; // end FiniteTrajectory4d implementation
    }

    public void addAct(Act act, double duration) {
        acts.add(ActConfiguration.create(act, duration));
    }

    //	private static class NullAct extends Act {
    //
    //	}

}
