package io.github.agentwise.swarmview.applications.trajectory;

import static io.github.agentwise.swarmview.applications.trajectory.TestUtils.testSpeedBounds;
import static io.github.agentwise.swarmview.applications.trajectory.TestUtils.verifyPositionFrequencyRadiusRelation;
import static io.github.agentwise.swarmview.applications.trajectory.TestUtils.verifyVelocityFrequencyRadiusRelation;

import org.junit.Test;

import io.github.agentwise.swarmview.applications.trajectory.BasicTrajectory;
import io.github.agentwise.swarmview.applications.trajectory.Trajectory1d;
import io.github.agentwise.swarmview.applications.trajectory.Trajectory2d;

/**
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public abstract class Periodic2DTest {
    protected final double lowFreq = 1 / 10;
    protected final double highFreq = 1.5;
    protected final double radius = 0.065;
    protected Trajectory2d highFrequencyCircle;
    protected Trajectory2d lowFrequencyCircle;

    @Test
    public void getTrajectoryLinearAbscissaTestFrequencyAndRadiusRelation() throws Exception {

        verifyPositionFrequencyRadiusRelation(highFreq, radius, new HighFreq1DAbscissa());
        verifyPositionFrequencyRadiusRelation(lowFreq, radius, new LowFreq1DAbscissa());
    }

    @Test
    public void getTrajectoryAbscissaVelocityTestFrequencyAndRadiusRelation() {
        testSpeedBounds(new HighFreq1DAbscissa(), BasicTrajectory.MAX_ABSOLUTE_VELOCITY);
        verifyVelocityFrequencyRadiusRelation(highFreq, new HighFreq1DAbscissa());

        testSpeedBounds(new LowFreq1DAbscissa(), BasicTrajectory.MAX_ABSOLUTE_VELOCITY);
        verifyVelocityFrequencyRadiusRelation(lowFreq, new LowFreq1DAbscissa());
    }

    @Test
    public void getTrajectoryLinearOrdinateTestFrequencyAndRadiusRelation() throws Exception {

        verifyPositionFrequencyRadiusRelation(highFreq, 0, new HighFreq1DOrdinate());
        verifyPositionFrequencyRadiusRelation(lowFreq, 0, new LowFreq1DOrdinate());
    }

    @Test
    public void getTrajectoryOrdinateVelocityTestFrequencyAndRadiusRelation() {
        testSpeedBounds(new HighFreq1DOrdinate(), BasicTrajectory.MAX_ABSOLUTE_VELOCITY);

        testSpeedBounds(new LowFreq1DOrdinate(), BasicTrajectory.MAX_ABSOLUTE_VELOCITY);
    }

    private class HighFreq1DAbscissa implements Trajectory1d {
        @Override
        public double getDesiredPosition(double timeInSeconds) {
            return highFrequencyCircle.getDesiredPositionAbscissa(timeInSeconds);
        }
    }

    private class HighFreq1DOrdinate implements Trajectory1d {
        @Override
        public double getDesiredPosition(double timeInSeconds) {
            return highFrequencyCircle.getDesiredPositionOrdinate(timeInSeconds);
        }
    }

    private class LowFreq1DAbscissa implements Trajectory1d {
        @Override
        public double getDesiredPosition(double timeInSeconds) {
            return lowFrequencyCircle.getDesiredPositionAbscissa(timeInSeconds);
        }
    }

    private class LowFreq1DOrdinate implements Trajectory1d {
        @Override
        public double getDesiredPosition(double timeInSeconds) {
            return lowFrequencyCircle.getDesiredPositionOrdinate(timeInSeconds);
        }
    }
}
