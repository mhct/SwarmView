package io.github.agentwise.swarmview.trajectory.applications.trajectory;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

public class ZigZagTrajectory4D extends BasicTrajectory implements FiniteTrajectory4d {
	
    private final Point4D srcpoint;
    private final Point4D targetpoint;
	
	private final double FRONT = -Math.PI/2;
	
	private final FiniteTrajectory4d target;


    public ZigZagTrajectory4D (Point4D srcpoint, Point4D targetpoint, int numberZZ, double zzDistance, double percentageVelocity) {
    	this.srcpoint = srcpoint;
    	this.targetpoint = targetpoint;
	    	
        TrajectoryComposite.Builder builder = TrajectoryComposite.builder();
        
        Point4D current = srcpoint;
        for (int i = 0; i < numberZZ; i++) {
        	
        	Point4D goEndMid = getIntermediatePoint(this.srcpoint, this.targetpoint, i+1, numberZZ);
        	Point4D goHalfMid = getIntermediatePoint(current, goEndMid, 1, 2);
        	Point4D baseGoLeft = getIntermediatePoint(current, goEndMid, 1, 4);
        	Point4D baseGoRight = getIntermediatePoint(current, goEndMid, 3, 4);
        	
        	Point4D goLeft = getZigZagPoint (baseGoLeft, this.targetpoint, zzDistance);
        	Point4D goRight = getZigZagPoint (baseGoRight, this.targetpoint, -zzDistance);
        	
    	    builder.addTrajectory(StraightLineTrajectory4D.createWithPercentageVelocity(current, goLeft, percentageVelocity));
    	    builder.addTrajectory(StraightLineTrajectory4D.createWithPercentageVelocity(goLeft, goHalfMid, percentageVelocity));
    	    builder.addTrajectory(StraightLineTrajectory4D.createWithPercentageVelocity(goHalfMid, goRight, percentageVelocity));
    	    builder.addTrajectory(StraightLineTrajectory4D.createWithPercentageVelocity(goRight, goEndMid, percentageVelocity));
    	    
    	    current = goEndMid;
        	
        }
        	
	    target = builder.build();

    }


	private Point4D getIntermediatePoint(Point4D start, Point4D end, int which, int totalSegments) {

    	double lambda = (double)which / totalSegments;
    	
		double x1 = start.getX();
    	double y1 = start.getY();
    	double z1 = start.getZ();
    	
    	double x2 = end.getX();
    	double y2 = end.getY();
    	double z2 = end.getZ();
    	
		return Point4D.create(x1 + lambda*(x2-x1), y1 + lambda*(y2-y1), z1 + lambda*(z2-z1), FRONT);

	}


	private Point4D getZigZagPoint(Point4D start, Point4D end, double d) {
		
		double x1 = start.getX();
    	double y1 = start.getY();
    	double z1 = start.getZ();
    	
    	double x2 = end.getX();
    	double y2 = end.getY();
    	double z2 = end.getZ();
    	
    	double dx = x1-x2;
		double dy = y1-y2;
		double dist = Math.sqrt(dx*dx + dy*dy);
		dx /= dist;
		dy /= dist;
		
		double x3 = x1 + (d/2)*dy;
		double y3 = y1 - (d/2)*dx;
		double z3 = z1;
		
		return Point4D.create(x3,  y3,  z3,  this.FRONT);
						
	}


	@Override
	public double getTrajectoryDuration() {
	    return target.getTrajectoryDuration();
	}

	@Override
	public Pose getDesiredPosition(double timeInSeconds) {
	    return Pose.create(target.getDesiredPosition(timeInSeconds));
	}

}
