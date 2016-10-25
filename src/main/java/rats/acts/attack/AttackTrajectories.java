/**
 * 
 */
package rats.acts.attack;

import applications.trajectory.LineTrajectory;
import applications.trajectory.Trajectory4d;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import control.FiniteTrajectory4d;
import control.dto.Pose;

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
	private static final double TIME_TO_ENGAGE = 1;
	private static final double TIME_TO_RETREAT = 2;


	public AttackTrajectories (Point3D middle, double initialHeight, double attackHeight, double initialRadius, double attackRadius) {
		this.middle = middle;
		this.initialHeight = initialHeight;
		this.attackHeight = attackHeight;
		this.initialRadius = initialRadius;
		this.attackRadius = attackRadius;
	}
	
	public FiniteTrajectory4d getTrajectory (int idDrone, int numberOfDrones, int sequenceOfAttack) {

		// first we create a line trajectory from the current position to the position to start the attack
		
		Point3D startPositionAttack;
		double x = this.middle.getX() + this.initialRadius * Math.cos( idDrone * 2*Math.PI/numberOfDrones);
		double y = this.middle.getY() + this.initialRadius * Math.sin( idDrone * 2*Math.PI/numberOfDrones);
		double z = this.initialHeight;
		startPositionAttack = Point3D.create(x, y, z);
		
		Point3D attackingPosition;
		x = this.middle.getX() + this.attackRadius * Math.cos( idDrone * 2*Math.PI/numberOfDrones);
		y = this.middle.getY() + this.attackRadius * Math.sin( idDrone * 2*Math.PI/numberOfDrones);
		z = this.attackHeight;
		attackingPosition = Point3D.create(x, y, z);
		
		
		double startTimeAttack = 0 + sequenceOfAttack * TIME_BETWEEN_INITIATING_TO_GO_TO_START_ATTACK_POSITION;
		double endTimeAttack   = startTimeAttack + TIME_TO_ENGAGE;
		double endTimeRetreat   = endTimeAttack + TIME_TO_RETREAT;
		
		Trajectory4d line = null;
		try {
			line = new LineTrajectory (startPositionAttack, attackingPosition, startTimeAttack, endTimeAttack);
			line = new LineTrajectory (attackingPosition, startPositionAttack, endTimeAttack, endTimeRetreat);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return line;
	}

}
