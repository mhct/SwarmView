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
	float zoom = 1.0f;
	final int displayDimensionX = 1024;
	final int displayDimensionY = 800;
	
	public static void main(String[] args) {
		PApplet.main(RatsView.class);
	}
	
	@Override
	public void settings() {
//		fullScreen();
		size(displayDimensionX, displayDimensionY, P3D);
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
		zoom = constrain(zoom, 0.3f, 4.0f);
		scale(zoom);
		
		/**
		 * Rotation of the screen
		 */
		float rotx = (2*mouse.x/(float) displayDimensionX)*-2*PI+PI;
		float roty = (2*mouse.y/(float) displayDimensionY)*-2*PI-PI;
		float rotz = rotzfactor*PI/36;
		
		pushMatrix();
		
		translate(width/(2*zoom), height/(2*zoom), rotz/zoom);
		rotateX(roty);  
		rotateZ(rotx);  
		
		pushMatrix();
		strokeWeight(2.0f);
		translate(0, 0, 100);
		box(width/1.2f, width/1.2f, 200);
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
	
	public void mouseWheel(MouseEvent event) {
		if(event.getCount() >= 0) { 
		    zoom += 0.05; 
		  } 
		  else {
		    zoom -= 0.05; 
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
	}

}
