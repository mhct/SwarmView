package io.github.agentwise.swarmview.trajectory.rats;

import static io.github.agentwise.swarmview.trajectory.control.DroneName.Fievel;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Juliet;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Nerve;
import static io.github.agentwise.swarmview.trajectory.control.DroneName.Romeo;

import java.util.ArrayList;
import java.util.List;

import io.github.agentwise.swarmview.trajectory.control.Act;
import io.github.agentwise.swarmview.trajectory.control.ActConfiguration;
import io.github.agentwise.swarmview.trajectory.control.Choreography;
import io.github.agentwise.swarmview.trajectory.control.ChoreographyView;
import io.github.agentwise.swarmview.trajectory.control.DronePositionConfiguration;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.operationaltests.BasicCirclesAct;

/**
 * Defines the Circle Operational Test
 * 
 * @author Mario h.c.t.
 *
 */
public class OTCircleShow {
	public final static double YAW = -Math.PI/2.0;
	private static ActConfiguration config;

	static {
		final List<DronePositionConfiguration> introPositions = new ArrayList<>();
		introPositions.add(DronePositionConfiguration.create(Nerve, Pose.create(4.0, 1.0, 1.0, YAW), Pose.create(4.0, 3.0, 1.0, YAW)));
		introPositions.add(DronePositionConfiguration.create(Romeo,  Pose.create(2.0, 3.0, 1.0, YAW), Pose.create(2.0, 3.0, 1.0, YAW)));
		introPositions.add(DronePositionConfiguration.create(Juliet, Pose.create(4.0, 5.0, 1.0, YAW), Pose.create(4.0, 5.0, 1.0, YAW)));
		introPositions.add(DronePositionConfiguration.create(Fievel,  Pose.create(6.0, 3.0, 1.0, YAW), Pose.create(6.0, 3.0, 1.0, YAW)));
		
		config = ActConfiguration.create("Circle", introPositions);
	}
	
	public static ChoreographyView createCircleChoreography() {
		Act circle = BasicCirclesAct.create(config);
        circle.lockAndBuild();

        final Choreography choreo = Choreography.create(config.numberDrones());
        choreo.addAct(circle);
		return choreo;
	}

	public static ChoreographyView createUpDown() {
		Act circle = BasicCirclesAct.create(config);
		circle.lockAndBuild();
		
		final Choreography choreo = Choreography.create(config.numberDrones());
		choreo.addAct(circle);
		return choreo;
	}
}
