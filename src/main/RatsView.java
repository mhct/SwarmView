package main;

import applications.trajectory.CircleTrajectory4D;
import applications.trajectory.geom.point.Point3D;
import control.Trajectory4d;
import processing.core.PApplet;
import processing.event.MouseEvent;

import java.awt.MouseInfo;
import java.awt.Point; 

public class RatsView extends PApplet {
	private final int NUMBER_DRONES = 1;
	Drone[] drones = new Drone[NUMBER_DRONES];
	private float timeStep;
	int rotzfactor = 0;
	float zoom = 0;
	
	public static void main(String[] args) {
		PApplet.main(RatsView.class);
	}
	
	@Override
	public void settings() {
//		fullScreen();
		size(600, 600, P3D);
	}
	
	@Override
	public void setup() {
		fill(255);
		timeStep = 0;
		
		Trajectory4d trajectory = CircleTrajectory4D.builder()
				.setLocation(Point3D.create(0, 0, 100))
				.setRadius(50)
				.setFrequency(0.008)
				.build();
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i] = new Drone(this, trajectory);
		}
		
	}
	
	@Override
	public void draw() {
		background(0);
		
		Point mouse = MouseInfo.getPointerInfo().getLocation();
		/**
		 * Rotation on the screen
		 */
//		float rotx = (mouseY/600.0f)*-2*PI+PI;
//		float roty = (mouseX/600.0f)*2*PI-PI;
//		float rotz = rotzfactor*PI/36;
		float rotx = (mouse.y/600.0f)*-2*PI+PI;
		float roty = (mouse.x/600.0f)*2*PI-PI;
		float rotz = rotzfactor*PI/36;
		
//		translate(0, 0, rotz);
		
		pushMatrix();
		
//		camera(mouseX, height/2, (height/2) / tan(PI/6), width/2, height/2, 0, 0, 1, 0);
//		scale(zoom, zoom, zoom not working
		translate(width/2, height/2);
		rotateX(rotx);  //
		rotateY(roty);  // rotate drawing coordinates according to user input variables
		rotateZ(rotz);  //
		box(250);
		
		pushMatrix();
		scale(350, 350, 350);
		cube();
		popMatrix();
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i].displayNext(timeStep);
		}
		
		popMatrix();
		
		
		timeStep = timeStep + 1.0f;
	}
	
	@Override
	public void keyPressed() {
		if (key == CODED) {
			if (keyCode == UP) {
				rotzfactor++;
				
		    }
			else if (keyCode == DOWN) {
				rotzfactor--;
		    	}
		}
	}
	
	public void mouseWheel(MouseEvent event) {
		zoom += event.getCount();
	}
	
	public void cube() {
		// Front
			noStroke();
		  beginShape(QUADS);
		  fill(255, 255, 0, 100);
		  vertex(-1, -1,  1);
		  vertex( 1, -1,  1);
		  vertex( 1,  1,  1);
		  vertex(-1,  1,  1);
		  endShape();
		  // Back
//		  beginShape(QUADS);
//		  fill(255,255,0);
//		  vertex( 1, -1, -1);
//		  vertex(-1, -1, -1);
//		  vertex(-1,  1, -1);
//		  vertex( 1,  1, -1);
//		  endShape();
//		  // Bottom
//		  beginShape(QUADS);
//		  fill( 255,0,255);
//		  vertex(-1,  1,  1);
//		  vertex( 1,  1,  1);
//		  vertex( 1,  1, -1);
//		  vertex(-1,  1, -1);
//		  endShape();
//		  // Top
//		  beginShape(QUADS);
//		  fill(0,255,0);
//		  vertex(-1, -1, -1);
//		  vertex( 1, -1, -1);
//		  vertex( 1, -1,  1);
//		  vertex(-1, -1,  1);
//		  endShape();
//		  // Right
//		  beginShape(QUADS);
//		  fill(0,0,255);
//		  vertex( 1, -1,  1);
//		  vertex( 1, -1, -1);
//		  vertex( 1,  1, -1);
//		  vertex( 1,  1,  1);
//		  endShape();
//		  // Left
//		  beginShape(QUADS);
//		  fill(0,255,255);
//		  vertex(-1, -1, -1);
//		  vertex(-1, -1,  1);
//		  vertex(-1,  1,  1);
//		  vertex(-1,  1, -1);
//		  endShape();
	}

}
