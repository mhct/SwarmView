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
		zoom = constrain(zoom, 1.0f, 2.0f);

		scale(zoom);
		/**
		 * Rotation on the screen
		 */
//		float rotx = (mouseY/600.0f)*-2*PI+PI;
//		float roty = (mouseX/600.0f)*2*PI-PI;
//		float rotz = rotzfactor*PI/36;
		float rotx = (2*mouse.x/600.0f)*-2*PI+PI;
		float roty = (2*mouse.y/600.0f)*-2*PI-PI;
		float rotz = rotzfactor*PI/36;
		
//		translate(0, 0, rotz);
		
		pushMatrix();
		
		//TODO fiz dimensions to dimensions of the room. Check coordinates from trajectory, and units
//		camera(mouseX, height/2, (height/2) / tan(PI/6), width/2, height/2, 0, 0, 1, 0);
//		scale(zoom, zoom, zoom not working
		translate(width/2, height/2);
		rotateX(roty);  
		rotateZ(rotx);  
		
		pushMatrix();
		strokeWeight(2.0f);
		translate(0, 0, 100);
		box(width/2, width/2, 200);
		popMatrix();
		
		pushMatrix();
		scale(700, 700, 700);
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
		if(event.getCount() >= 0) { 
		    zoom += 0.005; 
		  } 
		  else {
		    zoom -= 0.01; 
		  }
	}
	
	public void cube() {
		// Room floor
		noStroke();
		beginShape(QUADS);
		fill(255, 255, 0, 100);
		vertex(-1, -1,  0);
		vertex( 1, -1,  0);
		vertex( 1,  1,  0);
		vertex(-1,  1,  0);
		endShape();
		
		// Room back (x,z) plane
		noStroke();
		beginShape(TRIANGLE);
		fill(255, 0, 255, 100);
		vertex(-1, -1,  0);
		vertex( 0, -1,  1);
		vertex( 1,  -1,  0);
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
