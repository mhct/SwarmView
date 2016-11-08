package io.github.agentwise.rats.acts.taming;

import org.junit.Test;

import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.dto.Pose;
import io.github.agentwise.rats.acts.taming.TamingAct;
import io.github.agentwise.rats.acts.taming.TamingAct.Particle;

public class TamingActTest {

	@Test
	public void test() {
		Particle p = new TamingAct.Particle(Pose.create(3, 4, 0, 0));
		p.moveAway(Point4D.create(0, 0, 0, 0), 1, 1);
		Pose res = p.getTrajectory().getDesiredPosition(1);
		System.out.println(res);
	}

}
