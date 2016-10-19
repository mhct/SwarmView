package main;

import control.Trajectory4d;
import processing.core.PApplet;
import processing.event.KeyEvent;
import processing.event.MouseEvent;

import java.awt.MouseInfo;
import java.awt.Point;

import applications.trajectory.CircleTrajectory4D;
import applications.trajectory.NerveTrajectoryIntroduction;
import applications.trajectory.geom.point.Point3D; 

public class RatsView extends PApplet {
	private static final float MAX_ZOOM = 4.0f;
	private static final float MIN_ZOOM = 0.3f;
	private final int NUMBER_DRONES = 1;
	Drone[] drones = new Drone[NUMBER_DRONES];
	int rotzfactor = 0;
	float zoom = 1.0f;
	final int displayDimensionX = 1024;
	final int displayDimensionY = 800;
	private boolean mouseActive = true;
	private int lastMouseX;
	private int lastMouseY;
	private float lastZoom;
	private int initialTime = 0;
	private float rotz;
	
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
		
		/**
		 * Add trajectories here
		 */
//        Trajectory4d trajectory = ;
        

		NerveTrajectoryIntroduction nerve;
		try {
			nerve = new NerveTrajectoryIntroduction();
	        drones[0] = new Drone(this, nerve);
		} catch (Exception e) {
			e.printStackTrace();
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
		strokeWeight(2.0f);
		translate(0, 0, 200);
		box(800, 800, 400);
		popMatrix();

		drawStage();
		
		
		pushMatrix();
		translate(-400, -400, 0);
		text("Origin", 0.0f, 0.0f, 0.0f);
		int currentTime = millis();
		if (initialTime  == 0) {
			initialTime = millis();
		}
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i].displayNext((currentTime-initialTime)/1000.0f);
		}
		popMatrix();
		
		popMatrix();
	}
	
	/**
	 * Positions the Scene view according to the last mouse movement
	 * 
	 */
	private void positionView(float x, float y, float zoom) {
		zoom = constrain(zoom, MIN_ZOOM, MAX_ZOOM);
		scale(zoom);
	
		/**
		 * Rotation of the screen
		 */
		float rotx = (2*x/(float) displayDimensionX)*-2*PI+PI;
		float roty = (2*y/(float) displayDimensionY)*-2*PI-PI;
		rotz = rotzfactor*PI/36;
	
		pushMatrix();
		translate(width/(2*zoom), height/(2*zoom), rotz/zoom);
		rotateX(roty);  
		rotateZ(rotx);  
	}
	
	public void mouseWheel(MouseEvent event) {
		if(event.getCount() >= 0 && zoom <= MAX_ZOOM) { 
		    zoom += 0.05;
		  } 
		  else if (zoom >= MIN_ZOOM) {
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
		pushMatrix();
		scale(700, 700, 700);
		
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
		
		popMatrix();
	}

}
