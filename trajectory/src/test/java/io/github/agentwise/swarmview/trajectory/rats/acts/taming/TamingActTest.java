package io.github.agentwise.swarmview.trajectory.rats.acts.taming;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import org.junit.Test;

public class TamingActTest {

	@Test
	public void test() {
		TamingAct.Particle p = new TamingAct.Particle(Pose.create(3, 4, 0, 0));
		p.moveAway(Point4D.create(0, 0, 0, 0), 1, 1);
		Pose res = p.getTrajectory().getDesiredPosition(1);
		System.out.println(res);
	}

}
