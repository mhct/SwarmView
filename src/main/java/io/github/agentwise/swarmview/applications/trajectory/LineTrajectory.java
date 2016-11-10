package io.github.agentwise.swarmview.applications.trajectory;

import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.control.dto.Pose;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.github.agentwise.swarmview.control.FiniteTrajectory4d;

public class LineTrajectory implements FiniteTrajectory4d {
	private static final Logger logger = LoggerFactory.getLogger(LineTrajectory.class);
	
	final 	static double MAX_SPEED = 2.5;
	final 	static double DEFAULT_SPEED = MAX_SPEED * 0.8;
	
	private Pose startPosition;
	private Pose endPosition;
	private double startTime;
	private double endTime;
	
	private double currentTime = -1;
	private Pose currentPosition = Pose.create(0, 0, 0, 0);
	
	public LineTrajectory (Pose startPosition, Pose endPosition, double startTime) throws Exception {
		this(startPosition, endPosition, startTime, startTime + (Pose.computeEuclideanDistance (startPosition, endPosition) / DEFAULT_SPEED));
	}
	
	public LineTrajectory (Pose startPosition, Pose endPosition, double startTime, double endTime) throws Exception {
		double requiredSpeed = Pose.computeEuclideanDistance(startPosition, endPosition) / (endTime - startTime);
		if (requiredSpeed >= MAX_SPEED) {
			throw new Exception("Unrealisticly fast trajectory: " + startPosition + " --> " + endPosition
					+ "(minimum time required is " + (Pose.computeEuclideanDistance(startPosition, endPosition) / MAX_SPEED) +  ")");
		}
		this.startPosition = startPosition;
		this.endPosition = endPosition;
		this.startTime = startTime;
		this.endTime = endTime;
		this.currentTime = startTime;
		this.currentPosition = startPosition;
	}
	
	protected void calcDesiredPosition (double timeInSeconds) {
		logger.trace("time is " + timeInSeconds);
		
		if ( timeInSeconds < 0) {
			// throw new IllegalArgumentException ("Position requested for a time out of bound of this trajectory (" + timeInSeconds + ")");
			throw new IllegalArgumentException ("Position requested for a negative time (" + timeInSeconds + ")");
		}
		
		if (timeInSeconds == this.currentTime) {
			//
		} else {
			this.currentTime = timeInSeconds;
			
			if (timeInSeconds < this.startTime) {
				this.currentPosition = this.startPosition;
			} else if (timeInSeconds >= this.endTime) {
				this.currentPosition = this.endPosition;
			} else {
				double progress = (timeInSeconds - this.startTime) / (this.endTime - this.startTime);
				Point3D startPos = Point3D.create(this.startPosition.x(), this.startPosition.y(), this.startPosition.z());
				Point3D endPos = Point3D.create(this.endPosition.x(), this.endPosition.y(), this.endPosition.z());
				
				Point3D currentPos = Point3D.plus(
								startPos
								,
								Point3D.scale(
										Point3D.minus(endPos, startPos)
										,
										progress)
								);
				this.currentPosition = Pose.create(currentPos.getX(), currentPos.getY(), currentPos.getZ(), 0);
			}
		}
		
	}
	
//	@Override
//	public double getDesiredPositionX(double timeInSeconds) {
//		this.calcDesiredPosition(timeInSeconds);
//		return this.currentPosition.getX();
//	}
//
//	@Override
//	public double getDesiredPositionY(double timeInSeconds) {
//		this.calcDesiredPosition(timeInSeconds);
//		return this.currentPosition.getY();
//	}
//
//	@Override
//	public double getDesiredPositionZ(double timeInSeconds) {
//		this.calcDesiredPosition(timeInSeconds);
//		return Math.min(8.0-this.getDesiredPositionX(timeInSeconds), this.currentPosition.getZ());
//	}
//
//	@Override
//	public double getDesiredAngleZ(double timeInSeconds) {
//		// TODO Auto-generated method stub
//		return 0;
//	}

	public boolean isActive(double timeInSeconds) {
		if ((timeInSeconds >= this.startTime) && (timeInSeconds <= this.endTime))
			return true;
		else
			return false;
	}

	public boolean startsAfter(double timeInSeconds) {
		if (timeInSeconds < this.startTime)
			return true;
		else
			return false;	
	}

	@Override
	public double getTrajectoryDuration() {
		// TODO Auto-generated method stub
		return this.endTime - this.startTime;
	}

	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
		this.calcDesiredPosition(timeInSeconds);
		return this.currentPosition;
	}

}
