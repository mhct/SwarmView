package io.github.agentwise.swarmview.applications.trajectory;

import io.github.agentwise.swarmview.control.FiniteTrajectory4d;

/**
 * Trajectory server instances offer fully built finite trajectories.
 *
 * @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be)
 */
public interface TrajectoryServer {

  /** @return A fully built finite trajectory in 4D. */
  FiniteTrajectory4d getConcreteTrajectory();
}
