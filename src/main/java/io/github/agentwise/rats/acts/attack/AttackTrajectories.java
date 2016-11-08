/**
 * 
 */
package io.github.agentwise.rats.acts.attack;

import io.github.agentwise.applications.trajectory.HoldPositionTrajectory4D;
import io.github.agentwise.applications.trajectory.LineTrajectory;
import io.github.agentwise.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.applications.trajectory.Trajectory4d;
import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite;
import io.github.agentwise.applications.trajectory.composites.TrajectoryComposite.Builder;
import io.github.agentwise.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;

/**
 * @author tom
 *
 */
public class AttackTrajectories {

	private Point3D middle;
	
	private double initialHeight;
	private double attackHeight;
	
	private double initialRadius;
	private double attackRadius;
	
	
	private static final double TIME_BETWEEN_INITIATING_TO_GO_TO_START_ATTACK_POSITION = 1;
	private static final double PERCENTAGE_SPEED_TO_ENGAGE = 1;
	private static final double PERCENTAGE_SPEED_TO_RETREAT = 0.6;


	public AttackTrajectories (Point3D middle, double initialHeight, double attackHeight, double initialRadius, double attackRadius) {
		this.middle = middle;
		this.initialHeight = initialHeight;
		this.attackHeight = attackHeight;
		this.initialRadius = initialRadius;
		this.attackRadius = attackRadius;
	}
	
	public FiniteTrajectory4d getTrajectory (int idDrone, int numberOfDrones, int sequenceOfAttack) {

		// first we create a line trajectory from the current position to the position to start the attack
		
		Point4D startPositionAttack;
		double x = this.middle.getX() + this.initialRadius * Math.cos( idDrone * 2*Math.PI/numberOfDrones);
		double y = this.middle.getY() + this.initialRadius * Math.sin( idDrone * 2*Math.PI/numberOfDrones);
		double z = this.initialHeight;
		startPositionAttack = Point4D.create(x, y, z, 0);
		
		Point4D attackingPosition;
		x = this.middle.getX() + this.attackRadius * Math.cos( idDrone * 2*Math.PI/numberOfDrones);
		y = this.middle.getY() + this.attackRadius * Math.sin( idDrone * 2*Math.PI/numberOfDrones);
		z = this.attackHeight;
		attackingPosition = Point4D.create(x, y, z, 0);
		
		
		double startTimeAttack = 0.01 + sequenceOfAttack * TIME_BETWEEN_INITIATING_TO_GO_TO_START_ATTACK_POSITION;
		// double endTimeAttack   = startTimeAttack + TIME_TO_ENGAGE;
		// double endTimeRetreat   = endTimeAttack + TIME_TO_RETREAT;
		
		Builder trajectoryBuilder = TrajectoryComposite.builder();
		StraightLineTrajectory4D line;
		
		// First hover until it's your turn...
		
		trajectoryBuilder.addTrajectory(HoldPositionTrajectory4D.createFromPosition4D(startPositionAttack))
						.withDuration(startTimeAttack);
		
		// Now GO ATTACK !!
		
		line = StraightLineTrajectory4D.createWithPercentageVelocity(startPositionAttack, attackingPosition, PERCENTAGE_SPEED_TO_ENGAGE);
		trajectoryBuilder.addTrajectory(line);
		
		// And retreat...
		
		line = StraightLineTrajectory4D.createWithPercentageVelocity(attackingPosition, startPositionAttack, PERCENTAGE_SPEED_TO_RETREAT);
		trajectoryBuilder.addTrajectory(line);
		
		return trajectoryBuilder.build();
	}

}
