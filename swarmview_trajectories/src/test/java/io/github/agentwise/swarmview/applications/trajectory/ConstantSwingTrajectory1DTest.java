package io.github.agentwise.swarmview.applications.trajectory;

import org.junit.Before;
import org.junit.Test;

import io.github.agentwise.swarmview.applications.trajectory.ConstantSwingTrajectory1D;
import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point4D;

/** @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be) */
public class ConstantSwingTrajectory1DTest extends Periodic1DTest {

  @Before
  public void setup() {
    highFrequencyCircle = new ConstantSwingTrajectory1D(Point4D.origin(), radius, highFreq, phase);
    lowFrequencyCircle = new ConstantSwingTrajectory1D(Point4D.origin(), radius, lowFreq, phase);
  }

  @Test(expected = IllegalArgumentException.class)
  public void testConstructorTooHighSpeedRate() {
    new ConstantSwingTrajectory1D(Point4D.origin(), 1, 1, 0);
  }
}
