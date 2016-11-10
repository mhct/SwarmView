
package io.github.agentwise.swarmview.applications.trajectory;

import io.github.agentwise.swarmview.control.FiniteTrajectory4d;
import javax.annotation.Generated;

@Generated("com.google.auto.value.processor.AutoValueProcessor")
 final class AutoValue_CollisionDetector_Collision extends CollisionDetector.Collision {

  private final double timePoint;
  private final double actualDistance;
  private final FiniteTrajectory4d firstCollidingTrajectory;
  private final FiniteTrajectory4d secondCollidingTrajectory;

  AutoValue_CollisionDetector_Collision(
      double timePoint,
      double actualDistance,
      FiniteTrajectory4d firstCollidingTrajectory,
      FiniteTrajectory4d secondCollidingTrajectory) {
    this.timePoint = timePoint;
    this.actualDistance = actualDistance;
    if (firstCollidingTrajectory == null) {
      throw new NullPointerException("Null firstCollidingTrajectory");
    }
    this.firstCollidingTrajectory = firstCollidingTrajectory;
    if (secondCollidingTrajectory == null) {
      throw new NullPointerException("Null secondCollidingTrajectory");
    }
    this.secondCollidingTrajectory = secondCollidingTrajectory;
  }

  @Override
  public double getTimePoint() {
    return timePoint;
  }

  @Override
  public double getActualDistance() {
    return actualDistance;
  }

  @Override
  public FiniteTrajectory4d getFirstCollidingTrajectory() {
    return firstCollidingTrajectory;
  }

  @Override
  public FiniteTrajectory4d getSecondCollidingTrajectory() {
    return secondCollidingTrajectory;
  }

  @Override
  public String toString() {
    return "Collision{"
        + "timePoint=" + timePoint + ", "
        + "actualDistance=" + actualDistance + ", "
        + "firstCollidingTrajectory=" + firstCollidingTrajectory + ", "
        + "secondCollidingTrajectory=" + secondCollidingTrajectory
        + "}";
  }

  @Override
  public boolean equals(Object o) {
    if (o == this) {
      return true;
    }
    if (o instanceof CollisionDetector.Collision) {
      CollisionDetector.Collision that = (CollisionDetector.Collision) o;
      return (Double.doubleToLongBits(this.timePoint) == Double.doubleToLongBits(that.getTimePoint()))
           && (Double.doubleToLongBits(this.actualDistance) == Double.doubleToLongBits(that.getActualDistance()))
           && (this.firstCollidingTrajectory.equals(that.getFirstCollidingTrajectory()))
           && (this.secondCollidingTrajectory.equals(that.getSecondCollidingTrajectory()));
    }
    return false;
  }

  @Override
  public int hashCode() {
    int h = 1;
    h *= 1000003;
    h ^= (Double.doubleToLongBits(this.timePoint) >>> 32) ^ Double.doubleToLongBits(this.timePoint);
    h *= 1000003;
    h ^= (Double.doubleToLongBits(this.actualDistance) >>> 32) ^ Double.doubleToLongBits(this.actualDistance);
    h *= 1000003;
    h ^= this.firstCollidingTrajectory.hashCode();
    h *= 1000003;
    h ^= this.secondCollidingTrajectory.hashCode();
    return h;
  }

}
