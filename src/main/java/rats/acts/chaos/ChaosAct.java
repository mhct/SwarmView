package rats.acts.chaos;

import control.Act;
import control.ActConfiguration;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

public class ChaosAct extends Act {

    private ChaosAct(ActConfiguration configuration) {
        super(configuration);
    }

    /**
     * Adds all the movements of this act
     *
     * @return
     */
    public static Act create(ActConfiguration configuration) {
        Act act = new ChaosAct(configuration);

        try {
            act.addTrajectory(Nerve, AllDrones
                    .createNerveTrajectory(act.initialPosition(Nerve), act.finalPosition(Nerve),
                            act.getDuration()));
            act.addTrajectory(Romeo, AllDrones
                    .createRomeoTrajectory(act.initialPosition(Romeo), act.finalPosition(Romeo),
                            act.getDuration()));
            act.addTrajectory(Juliet, AllDrones
                    .createJulietTrajectory(act.initialPosition(Juliet), act.finalPosition(Juliet),
                            act.getDuration()));
            act.addTrajectory(Fievel, AllDrones
                    .createFievelTrajectory(act.initialPosition(Fievel), act.finalPosition(Fievel),
                            act.getDuration()));
            act.addTrajectory(Dumbo, AllDrones
                    .createDumboTrajectory(act.initialPosition(Dumbo), act.finalPosition(Dumbo),
                            act.getDuration()));
        } catch (Exception e) {
            e.printStackTrace();
        }

        return act;
    }

    //    private static FiniteTrajectory4d exampleLineTrajectory(Pose initialPosition,
    //            Pose finalPosition, double duration) {
    //        return StraightLineTrajectory4D
    //                .createWithCustomTravelDuration(Point4D.from(initialPosition),
    //                        Point4D.from(finalPosition), duration);
    //    }
}
