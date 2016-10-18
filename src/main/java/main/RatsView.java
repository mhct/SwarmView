package main;

import control.Trajectory4d;
import processing.core.PApplet;
import processing.event.KeyEvent;
import processing.event.MouseEvent;

import java.awt.MouseInfo;
import java.awt.Point;

import applications.trajectory.CircleTrajectory4D;
import applications.trajectory.geom.point.Point3D; 

public class RatsView extends PApplet {
	private final int NUMBER_DRONES = 4;
	Drone[] drones = new Drone[NUMBER_DRONES];
	private float timeStep;
	int rotzfactor = 0;
	float zoom = 1.0f;
	final int displayDimensionX = 1024;
	final int displayDimensionY = 800;
	private boolean mouseActive = true;
	private int lastMouseX;
	private int lastMouseY;
	private float lastZoom;
	
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
		
		/**
		 * Add trajectories here
		 */
//        Trajectory4d trajectory = ;
        
        rectMode(CORNER);
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i] = new Drone(this, CircleTrajectory4D.builder()
					.setLocation(Point3D.create(width/4, height/4, i*100 +40))
					.setRadius(i*50)
					.setFrequency(0.001)
					.build());
		}
		
	}
	
	@Override
	public void draw() {
		background(0);
		
		Point mouse = MouseInfo.getPointerInfo().getLocation();
		
		if (mouseIsActive()) {
			lastMouseX = mouse.x;
			lastMouseY = mouse.y;
			lastZoom = zoom;
		}
		
		positionView(lastMouseX, lastMouseY, lastZoom);
		pushMatrix();
		
		text("0, 0, 0", 0.0f, 0.0f, 0.0f);
		strokeWeight(2.0f);
//		translate(0, 0, 100);
		box(800, 800, 500);
		popMatrix();
		
		pushMatrix();
		scale(700, 700, 700);
		drawStage();
		popMatrix();
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i].displayNext(timeStep);
		}
		
		popMatrix();

		timeStep = timeStep + 1.0f;
	}
	
	/**
	 * Positions the Scene view according to the last mouse movement
	 * 
	 */
	private void positionView(float x, float y, float zoom) {
		zoom = constrain(zoom, 0.3f, 4.0f);
		scale(zoom);
	
		/**
		 * Rotation of the screen
		 */
		float rotx = (2*x/(float) displayDimensionX)*-2*PI+PI;
		float roty = (2*y/(float) displayDimensionY)*-2*PI-PI;
		float rotz = rotzfactor*PI/36;
	
		pushMatrix();
		translate(width/(2*zoom), height/(2*zoom), rotz/zoom);
		rotateX(roty);  
		rotateZ(rotx);  
	}
	
	public void mouseWheel(MouseEvent event) {
		if(event.getCount() >= 0) { 
		    zoom += 0.05; 
		  } 
		  else {
		    zoom -= 0.05; 
		  }
	}
	
	@Override
	public void keyPressed(KeyEvent event) {
		if (event.getKey() == 'z') {
			 mouseToggle();
		}
	}
	
	/**
	 * Activates or deactivates listening to mouse events
	 */
	private void mouseToggle() {
		if (mouseActive == true) {
			mouseActive = false;
		} else {
			mouseActive = true;
		}
	}
	
	private boolean mouseIsActive() {
		return mouseActive;
	}
	public void drawStage() {
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
