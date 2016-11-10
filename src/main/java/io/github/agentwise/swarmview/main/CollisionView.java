package io.github.agentwise.swarmview.main;

import io.github.agentwise.swarmview.applications.trajectory.geom.point.Point3D;

public class CollisionView {
	private final RatsView canvas;
	private final Point3D point;

	public CollisionView(RatsView canvas, Point3D point) {
		this.canvas = canvas;
		this.point = point;
	}
	
	public void displayNext() {
		canvas.pushMatrix();
		canvas.noFill();
		canvas.translate((float)point.getX()*100, (float)point.getY()*100, (float)point.getZ()*100);
		canvas.textSize(100);
//		canvas.text("X", 0, 0, 0);
		canvas.box(50);
		canvas.stroke(255);
		canvas.popMatrix();
	}
}
