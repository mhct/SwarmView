package applications.trajectory;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import applications.trajectory.geom.point.Point3D;
import control.Trajectory4d;

public class LineTrajectory implements Trajectory4d {
	private static final Logger logger = LoggerFactory.getLogger(LineTrajectory.class);
	final double MAX_SPEED = 2.5;
	
	private Point3D startPosition;
	private Point3D endPosition;
	private double startTime;
	private double endTime;
	
	private double currentTime = -1;
	private Point3D currentPosition = Point3D.origin();
	
	public LineTrajectory (Point3D startPosition, Point3D endPosition, double startTime, double endTime) throws Exception {
		double requiredSpeed = Point3D.distance(startPosition, endPosition) / (endTime - startTime);
		if (requiredSpeed >= MAX_SPEED) {
			throw new Exception("Unrealisticly fast trajectory: " + startPosition + " --> " + endPosition
					+ "(minimum time required is " + (Point3D.distance(startPosition, endPosition) / MAX_SPEED) +  ")");
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
				this.currentPosition = Point3D.plus(
										this.startPosition
										,
										Point3D.scale(
												Point3D.minus(this.endPosition, this.startPosition)
												,
												progress)
										);
			}
		}
		
	}
	@Override
	public double getDesiredPositionX(double timeInSeconds) {
		this.calcDesiredPosition(timeInSeconds);
		return this.currentPosition.getX();
	}

	@Override
	public double getDesiredPositionY(double timeInSeconds) {
		this.calcDesiredPosition(timeInSeconds);
		return this.currentPosition.getY();
	}

	@Override
	public double getDesiredPositionZ(double timeInSeconds) {
		this.calcDesiredPosition(timeInSeconds);
		return Math.min(8.0-this.getDesiredPositionX(timeInSeconds), this.currentPosition.getZ());
	}

	@Override
	public double getDesiredAngleZ(double timeInSeconds) {
		// TODO Auto-generated method stub
		return 0;
	}

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

}
