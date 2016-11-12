package io.github.agentwise.swarmview.trajectory.control.dto;

import com.google.auto.value.AutoValue;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;

@AutoValue
public abstract class Pose {

  public static Pose create(double x, double y, double z, double yaw) {
    return new AutoValue_Pose(x, y, z, yaw);
  }

  public static Pose create(Pose originalPose) {
    return new AutoValue_Pose(
        originalPose.x(), originalPose.y(), originalPose.z(), originalPose.yaw());
  }

  /** @return The X coordinate. */
  public abstract double x();

  /** @return The Y coordinate. */
  public abstract double y();

  /** @return The Z coordinate. */
  public abstract double z();

  /** @return The yaw angle orientation. */
  public abstract double yaw();

  /**
   * Computes the euclidean distance between two poses. The yaw is not taken into account.
   *
   * @param p1 the first pose
   * @param p2 the second pose
   * @return the euclidean distance between the two poses
   */
  public static double computeEuclideanDistance(Pose p1, Pose p2) {
    final double deltaX = p1.x() - p2.x();
    final double deltaY = p1.y() - p2.y();
    final double deltaZ = p1.z() - p2.z();

    return StrictMath.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
  }

  /**
   * Computes the 2d x-y euclidean distance between two poses.
   *
   * @param p1 the first pose
   * @param p2 the second pose
   * @return the euclidean distance between two poses on x-y plane
   */
  public static double compute2dEuclideanDistance(Pose p1, Pose p2) {
    final double deltaX = p1.x() - p2.x();
    final double deltaY = p1.y() - p2.y();

    return StrictMath.sqrt(deltaX * deltaX + deltaY * deltaY);
  }

  public static Pose createFromTrajectory(FiniteTrajectory4d trajectory, double timeInSecs) {
    return Pose.create(trajectory.getDesiredPosition(timeInSecs));
  }
}
