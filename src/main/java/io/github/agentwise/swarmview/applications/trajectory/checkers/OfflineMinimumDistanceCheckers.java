package io.github.agentwise.swarmview.applications.trajectory.checkers;


import com.google.common.base.MoreObjects;

import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.control.dto.Pose;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

/** @author Hoang Tung Dinh */
public class OfflineMinimumDistanceCheckers {
  private static final double DELTA_TIME = 0.001;

  private OfflineMinimumDistanceCheckers() {}

  /**
   * Checks whether there is any violation of the minimum distance constraint among violation
   * trajectories. This method will returns the first violation if there is at least one collision,
   * or return absent if there is no violation.
   *
   * @param trajectories the collection of all trajectories to be checked
   * @param minimumDistance the minimum distance in METERS between two trajectories
   * @return the first violation if there is at least on violation or absent if there is no
   *     violation
   */
  public static List<Optional<Violation>> checkMinimum3dDistanceConstraint(
      Collection<FiniteTrajectory4d> trajectories, double minimumDistance) {
	
	final List<Optional<Violation>> violations = new ArrayList<>();
    final Queue<FiniteTrajectory4d> trajectoryList = new LinkedList<>(trajectories);
    
    while (!trajectoryList.isEmpty()) {
      final FiniteTrajectory4d trajectory = trajectoryList.remove();
      final double duration = trajectory.getTrajectoryDuration();

      for (final FiniteTrajectory4d otherTrajectory : trajectoryList) {
        double t = 0;
        while (t <= duration) {
          final Pose firstPose = Pose.createFromTrajectory(trajectory, t);
          final Pose secondPose = Pose.createFromTrajectory(otherTrajectory, t);
          final double distance = Pose.computeEuclideanDistance(firstPose, secondPose);
          if (distance < minimumDistance) {
             violations.add(Optional.of(Violation.create(trajectory, otherTrajectory, t)));
          } else {
            t += DELTA_TIME;
          }
        }
      }
    }
    return violations;
  }
  
  public static Optional<Violation> checkMinimum3dDistanceConstraintAtTime(
		Collection<Pose> posesList, double minimumDistance, double time) {
	
	final Queue<Pose> poses = new LinkedList<>(posesList);
	
	while (!poses.isEmpty()) {
		final Pose pose = poses.remove();
		
		for (final Pose otherPose : poses) {
			final double distance = Pose.computeEuclideanDistance(pose, otherPose);
			if (distance < minimumDistance) {
				return Optional.of(Violation.create(pose, otherPose, time));
			}
		}
	}
	return Optional.empty();
  }

  /**
   * Checks whether there is any violation of the 2d x-y minimum distance constraint among violation
   * trajectories. This method will returns the first violation if there is at least one violation,
   * or return absent if there is no violation.
   *
   * @param trajectories the collection of all trajectories to be checked
   * @param minimumDistance the minimum distance between two trajectories
   * @return the first violation if there is at least on violation or absent if there is no
   *     violation
   */
  public static Optional<Violation> checkMinimum2dDistanceConstraint(
      Collection<FiniteTrajectory4d> trajectories, double minimumDistance) {
    final Queue<FiniteTrajectory4d> trajectoryList = new LinkedList<>(trajectories);

    while (!trajectoryList.isEmpty()) {
      final FiniteTrajectory4d trajectory = trajectoryList.remove();
      final double duration = trajectory.getTrajectoryDuration();

      for (final FiniteTrajectory4d otherTrajectory : trajectoryList) {
        double t = 0;
        while (t <= duration) {
          final Pose firstPose = Pose.createFromTrajectory(trajectory, t);
          final Pose secondPose = Pose.createFromTrajectory(otherTrajectory, t);
          final double distance = Pose.compute2dEuclideanDistance(firstPose, secondPose);
          if (distance < minimumDistance) {
            return Optional.of(Violation.create(trajectory, otherTrajectory, t));
          } else {
            t += DELTA_TIME;
          }
        }
      }
    }

    return Optional.empty();
  }

  public static final class Violation {
    private FiniteTrajectory4d firstTrajectory;
    private FiniteTrajectory4d secondTrajectory;
    private double collisionTimeInSecs;
	private Pose firstPose;
	private Pose secondPose;

    private Violation(
        FiniteTrajectory4d firstTrajectory,
        FiniteTrajectory4d secondTrajectory,
        double collisionTimeInSecs) {
      this.firstTrajectory = firstTrajectory;
      this.secondTrajectory = secondTrajectory;
      this.collisionTimeInSecs = collisionTimeInSecs;
    }

    private Violation(
    		Pose firstPose,
    		Pose secondPose,
    		double time) {
    	this.firstPose = firstPose;
    	this.secondPose = secondPose;
    	this.collisionTimeInSecs = time;
    }
    public static Violation create(
    		Pose pose, 
    		Pose otherPose, 
    		double time) {
		return new Violation(pose, otherPose, time);
	}

	public static Violation create(
        FiniteTrajectory4d firstTrajectory,
        FiniteTrajectory4d secondTrajectory,
        double collisionTimeInSecs) {
      return new Violation(firstTrajectory, secondTrajectory, collisionTimeInSecs);
    }

    @Override
    public String toString() {
      return MoreObjects.toStringHelper(this)
          .add("firstTrajectory", firstTrajectory)
          .add("secondTrajectory", secondTrajectory)
          .add("collisionTimeInSecs", collisionTimeInSecs)
          .toString();
    }
    
    public Pose getFirstPose() {
    	return this.firstPose;
    }
    
    public Pose getSecondPose() {
    	return this.secondPose;
    }
    
    public double getCollisionTimeInSecs() {
    	return collisionTimeInSecs;
    }
  }
}
