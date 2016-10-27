package rats.acts.chaos;

import com.google.common.collect.Lists;
import control.Act;
import control.ActConfiguration;
import control.dto.Pose;

import java.util.List;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;

public class ChaosAct extends Act {
    protected static final double orientation = -Math.PI / 2d;

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
        double[][] nerveWPCoords = { //start: 2,2,4 end: 6,6,2  cyan
                { 2.5, 1.5, 2.5 },
                { 3.5, 2.5, 3.0 },
                { 6.0, 1.9, 4 },
                { 5.0, 4.5, 2.5 }
        };
        double[][] romeoWPCoords = { //start: 1.1, 5.0, 1.5 end: 3.5, 4.0, 1.0  yellow
                { 0.8, 5.0, 1.2 },
                { 3.5, 4.0, 4.0 },
                { 1.1, 4.0, 1.5 },
                { 2.8, 4, 4 }
        };
        double[][] julietWPCoords = { //start: 4.9, 5.0, 1.5 end: 1,1,3   purple
                { 4.9, 5, 1.2 },
                { 2, 3, 3.0 },
                { 0.75, 1.6, 1.8 },
                { 1.0, 1.2, 2.5 }
        };
        double[][] fievelWPCoords = { //start: 5.0, 2.5, 1.0 end: 2, 5, 2   green
                { 6.0, 2.5, 2.0 },
                { 6.2, 6.0, 3.2 },
                { 6.2, 4.5, 4.0 },
                { 2.5, 5.0, 4.0 },
        };
        double[][] dumboWPCoords = { //start: 4.0, 3.5, 2.5 end: 1.5, 3, 1   blue
                { 1.5, 3.0, 1.0 },
                { 4.0, 3.5, 3 },
                { 1.5, 3.0, 3.0 },
                { 3.8, 2, 0, 2 }
        };

        List<Pose> nerveWPs = Lists
                .newArrayList(act.initialPosition(Nerve), coordToPose(nerveWPCoords[0]),
                        coordToPose(nerveWPCoords[1]), coordToPose(nerveWPCoords[2]),
                        coordToPose(nerveWPCoords[3]), act.finalPosition(Nerve));
        List<Pose> romeoWPs = Lists
                .newArrayList(act.initialPosition(Romeo), coordToPose(romeoWPCoords[0]),
                        coordToPose(romeoWPCoords[1]), coordToPose(romeoWPCoords[2]),
                        coordToPose(romeoWPCoords[3]), act.finalPosition(Romeo));
        List<Pose> julietWPs = Lists
                .newArrayList(act.initialPosition(Juliet), coordToPose(julietWPCoords[0]),
                        coordToPose(julietWPCoords[1]), coordToPose(julietWPCoords[2]),
                        coordToPose(julietWPCoords[3]), act.finalPosition(Juliet));
        List<Pose> fievelWPs = Lists
                .newArrayList(act.initialPosition(Fievel), coordToPose(fievelWPCoords[0]),
                        coordToPose(fievelWPCoords[1]), coordToPose(fievelWPCoords[2]),
                        coordToPose(fievelWPCoords[3]), act.finalPosition(Fievel));
        List<Pose> dumboWPs = Lists
                .newArrayList(act.initialPosition(Dumbo), coordToPose(dumboWPCoords[0]),
                        coordToPose(dumboWPCoords[1]), coordToPose(dumboWPCoords[2]),
                        coordToPose(dumboWPCoords[3]), act.finalPosition(Dumbo));

        try {
            act.addTrajectory(Nerve, AllDrones.createNerveTrajectory(nerveWPs, act.getDuration()));
            act.addTrajectory(Romeo, AllDrones.createRomeoTrajectory(romeoWPs, act.getDuration()));
            act.addTrajectory(Juliet,
                    AllDrones.createJulietTrajectory(julietWPs, act.getDuration()));
            act.addTrajectory(Fievel,
                    AllDrones.createFievelTrajectory(fievelWPs, act.getDuration()));
            act.addTrajectory(Dumbo, AllDrones.createDumboTrajectory(dumboWPs, act.getDuration()));
        } catch (Exception e) {
            e.printStackTrace();
        }

        return act;
    }

    private static Pose coordToPose(double[] coord) {
        return Pose.create(coord[0], coord[1], coord[2], orientation);
    }
}
