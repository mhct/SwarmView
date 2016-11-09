package io.github.agentwise.applications.trajectory;

import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.dto.Pose;

/** @author Kristof Coninx (kristof.coninx AT cs.kuleuven.be) */
public class HoldPositionTrajectory4D implements Trajectory4d {
  private final Point4D targetPoint;

  HoldPositionTrajectory4D(Point4D targetpoint) {
    this.targetPoint = targetpoint;
  }

  public static HoldPositionTrajectory4D create(Pose pose) {
	  return new HoldPositionTrajectory4D(Point4D.from(pose));
  }
  
  public static HoldPositionTrajectory4D createFromPosition4D(Point4D position) {
	  return new HoldPositionTrajectory4D(position);
  }
  
  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    return targetPoint.getX();
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    return targetPoint.getY();
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    return targetPoint.getZ();
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    return targetPoint.getAngle();
  }

  @Override
  public String toString() {
    return "HoldPositionTrajectory4D{" + "target point=" + targetPoint + '}';
  }
}
