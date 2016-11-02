package rats.acts.taming;

import org.junit.Test;

import applications.trajectory.geom.point.Point4D;
import control.dto.Pose;
import rats.acts.taming.TamingAct.Particle;

public class TamingActTest {

	@Test
	public void test() {
		Particle p = new TamingAct.Particle(Pose.create(3, 4, 0, 0));
		p.moveAway(Point4D.create(0, 0, 0, 0), 1, 1);
		Pose res = p.getTrajectory().getDesiredPosition(1);
		System.out.println(res);
	}

}
