package rats.acts.chaos;

import applications.trajectory.CollisionDetector;
import control.Act;
import control.ActConfiguration;
import control.DroneName;
import control.DronePositionConfiguration;
import control.FiniteTrajectory4d;
import control.dto.Pose;
import org.junit.Before;
import org.junit.Test;
import rats.acts.introduction.IntroductionAct;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static control.DroneName.Dumbo;
import static control.DroneName.Fievel;
import static control.DroneName.Juliet;
import static control.DroneName.Nerve;
import static control.DroneName.Romeo;
import static junit.framework.TestCase.assertEquals;

/**
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public class ChaosActTest {

    private static final double DISTANCE = 1;
    private Act chaos;

    @Before
    public void setUp() {
        List<DronePositionConfiguration> introPositions = new ArrayList<>();
        introPositions.add(DronePositionConfiguration
                .create(Nerve, Pose.create(7.0, 6.0, 1.0, 0.0), Pose.create(2.0, 2.0, 4.0, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Romeo, Pose.create(7.0, 5.0, 1.0, 0.0), Pose.create(1.1, 5.0, 1.5, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Juliet, Pose.create(1.0, 5.0, 1.0, 0.0), Pose.create(4.9, 5.0, 1.5, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Fievel, Pose.create(1.0, 6.0, 1.0, 0.0), Pose.create(5.0, 2.5, 1.0, 0.0)));
        introPositions.add(DronePositionConfiguration
                .create(Dumbo, Pose.create(2.0, 3.0, 1.0f, 0.0), Pose.create(4.0, 3.5, 2.5, 0.0)));
        ActConfiguration introConfiguration = ActConfiguration.create(introPositions);
        Act introduction = IntroductionAct.create(introConfiguration);
        List<DronePositionConfiguration> chaosPositions = new ArrayList<>();
        chaosPositions.add(DronePositionConfiguration
                .create(Nerve, introduction.finalPosition(Nerve), Pose

                        .create(6.0, 6.0, 2.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Romeo, introduction.finalPosition(Romeo), Pose.create(3.5, 4.0, 1.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Juliet, introduction.finalPosition(Juliet),
                        Pose.create(1.0, 1.0, 2.5, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Fievel, introduction.finalPosition(Fievel),
                        Pose.create(2.0, 5.0, 2.0, 0.0)));
        chaosPositions.add(DronePositionConfiguration
                .create(Dumbo, introduction.finalPosition(Dumbo), Pose.create(1.5, 3.0, 1.0, 0.0)));
        ActConfiguration chaosConfiguration = ActConfiguration.create(chaosPositions);
        this.chaos = ChaosAct.create(chaosConfiguration);
    }

    @Test
    public void testForCollisions() {
        CollisionDetector detector = new CollisionDetector(getAllTrajectories());
        List<CollisionDetector.Collision> collisions = detector.findCollisions();
        System.out.println(collisions);
        assertEquals(0, collisions.size());

    }

    private List<FiniteTrajectory4d> getAllTrajectories() {
        return Stream.of(DroneName.values()).sequential().map(name -> chaos.getTrajectory(name))
                .collect(Collectors.toList());
    }

}