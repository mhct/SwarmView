package main;

import applications.trajectory.NerveTrajectoryIntroduction;
import control.FiniteTrajectory4d;
import processing.core.PApplet;
import processing.event.KeyEvent;
import processing.event.MouseEvent;
import show.TwinDrones;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class RatsView extends PApplet {
	private static final float MAX_ZOOM = 4.0f;
	private static final float MIN_ZOOM = 0.3f;
	private final int NUMBER_DRONES = 3;
	Drone[] drones = new Drone[NUMBER_DRONES];
	int rotzfactor = 0;
	float zoom = 1.0f;
	final int displayDimensionX = 1024;
	final int displayDimensionY = 800;
	private boolean mouseActive = true;
	private boolean timerActive = true;
	private boolean simulationActive = true;
	
	private int lastMouseX;
	private int lastMouseY;
	private float lastZoom;
	private int initialTime = 0;
	private float rotz;
	private int lastTimeStep;
	
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
		
		initializeTrajectories();
	}
	
	private void initializeTrajectories() {
		initialTime = 0;
		/**
		 * Add trajectories here
		 */
        List<FiniteTrajectory4d> trajectories = new ArrayList<>();
		try {
			FiniteTrajectory4d temp = new NerveTrajectoryIntroduction();
			trajectories.add(temp);
	        drones[0] = new Drone(this, temp, color(0, 244, 200), 1);
	        
	        temp = TwinDrones.createRomeoTrajectory();
	        trajectories.add(temp);
            drones[1] = new Drone(this, temp, color(200, 100, 10), 20);
            
            temp = TwinDrones.createJulietTrajectory();
            trajectories.add(temp);
            drones[2] = new Drone(this, temp, color(200, 0, 200), 50);

        } catch (Exception e) {
            e.printStackTrace();
        }

        /**
         * Safety checks for collision between drones
         */
        //		OfflineMinimumDistanceCheckers.checkMinimum3dDistanceConstraint(trajectories, 1.0);
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
		
		int timeStep;
		if (simulationIsActive()) {
			int currentTime = millis();
			if (initialTime  == 0) {
				initialTime = millis();
			}
			
			timeStep = currentTime-initialTime;
			lastTimeStep = timeStep;
		} else {
			timeStep = lastTimeStep;
		}
		
		if (timerIsActive()) {
			drawTimer(timeStep);
		}
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i].displayNext((timeStep)/1000.0f);
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
	
	/**
	 * Handles user input via the keyboard
	 */
	@Override
	public void keyPressed(KeyEvent event) {
		if (event.getKey() == 'z') {
			 mouseToggle();
		}
		
		if (event.getKey() == 't') {
			timerToggle();
		}
		
		if (event.getKey() == ' ') {
			pauseToggle();
		}
		
		if (event.getKey() == 'r') {
			initializeTrajectories();
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
	
	/**
	 * Activates/deactivates the timer
	 */
	private void timerToggle() {
		if (timerActive  == true) {
			timerActive = false;
		} else {
			timerActive = true;
		}
	}
	
	/**
	 * Activates/deactivates the simulation
	 */
	private void pauseToggle() {
		if (simulationActive == true) {
			simulationActive = false;
		} else {
			simulationActive = true;
		}
	}
	
	private boolean mouseIsActive() {
		return mouseActive;
	}
	
	private boolean timerIsActive() {
		return timerActive;
	}
	
	private boolean simulationIsActive() {
		return simulationActive;
	}
	/**
	 * Draws timer
	 * @param time in milliseconds
	 */
	public void drawTimer(double time) {
		pushMatrix();
		rotateX(-PI/2);
		fill(255);
		textSize(72);
		
		int seconds = (int) (time / 1000) % 60 ;
		int minutes = (int) ((time / (1000*60)) % 60);
		int milliseconds = (int) (time % 1000);
		text(String.format("%02d' %02d\" %03d", minutes, seconds, milliseconds), -100.0f, -450.0f, 0.0f);
		popMatrix();
	}
	
	/**
	 * Draws the stage on the screen
	 */
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
		
		// TODO draw markers for the x,y,z coordinates on the corners
		
		popMatrix();
	}
}
