package io.github.agentwise.main;

import static io.github.agentwise.control.DroneName.Dumbo;
import static io.github.agentwise.control.DroneName.Fievel;
import static io.github.agentwise.control.DroneName.Juliet;
import static io.github.agentwise.control.DroneName.Nerve;
import static io.github.agentwise.control.DroneName.Romeo;

import java.awt.MouseInfo;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import io.github.agentwise.applications.trajectory.checkers.OfflineMinimumDistanceCheckers;
import io.github.agentwise.applications.trajectory.checkers.OfflineMinimumDistanceCheckers.Violation;
import io.github.agentwise.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.control.ChoreographyView;
import io.github.agentwise.control.FiniteTrajectory4d;
import io.github.agentwise.control.dto.Pose;
import processing.core.PApplet;
import processing.event.KeyEvent;
import processing.event.MouseEvent;

public class RatsView extends PApplet {
    private static final float MAX_ZOOM = 4.0f;
    private static final float MIN_ZOOM = 0.3f;
    DroneView[] drones;
    int rotzfactor = 0;
    float zoom = 1.0f;
    final int displayDimensionX = 1024;
    final int displayDimensionY = 800;
    private boolean mouseActive = true;
    private boolean timerActive = true;
    private boolean droneNameActive = true;
    private boolean simulationActive = false;

    private int lastMouseX;
    private int lastMouseY;
    private float lastZoom;
    private int initialTime = 0;
    private float rotz;
    private int lastTimeStep;

    ChoreographyView choreo;
	private int deltaTime = 0;
	private int deltaTimeTemp;
	private List<CollisionView> collisions;

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
        initialTime = millis(); //TODO add separate method to reset the view parameters
        deltaTime = 0;

        choreo = RatsShow.createChoreography();
        List<FiniteTrajectory4d> trajectories = choreo.getAllTrajectories();
//        List<Optional<Violation>> violations = OfflineMinimumDistanceCheckers.checkMinimum3dDistanceConstraint(trajectories, 0.5);
//        if (!violations.isEmpty()) {
//        	for (Optional<Violation> violation: violations) {
//        		System.out.println("Trajectories have a collision at: " + violation);
//        	}
//        }
        choreo = RatsShow.createChoreography();

        //
        //Configures the view
        //
        drones = new DroneView[choreo.getNumberDrones()];
        collisions = new ArrayList<>();
        
        drones[0] = new DroneView(this, choreo.getFullTrajectory(Nerve), color(0, 200, 200),
                10, Nerve.toString());   //cyan
        drones[1] = new DroneView(this, choreo.getFullTrajectory(Romeo), color(200, 200, 0),
                10, Romeo.toString());  //yellow
        drones[2] = new DroneView(this, choreo.getFullTrajectory(Juliet), color(200, 0, 200),
                10, Juliet.toString()); //purple
        drones[3] = new DroneView(this, choreo.getFullTrajectory(Fievel), color(0, 255, 0),
                10, Fievel.toString());  //green
        drones[4] = new DroneView(this, choreo.getFullTrajectory(Dumbo), color(0, 0, 250),
                10, Dumbo.toString());   //blue
        /**
         * Safety checks for collision between drones
         */
    }

	
	@Override
	public void draw() {
		background(0);
		
		Point mouse = MouseInfo.getPointerInfo().getLocation();
		
		if (isMouseActive()) {
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
		
		int timeStep = getCurrentTimeStep();
		if (isTimerActive()) {
			drawTimer(timeStep, choreo.getCurrentActName(timeStep/1000.0f));
		}
		List<Pose> posesCurrentTime = new ArrayList<Pose>();
		for (int i=0; i<choreo.getNumberDrones(); i++) {
			Pose pose = drones[i].displayNext((timeStep)/1000.0f);
			posesCurrentTime.add(pose);
		}

		// show collisions
		for (CollisionView coll: collisions) {
			coll.displayNext();
		}
		//
		// show possible collision
		//
		Optional<Violation> violation = OfflineMinimumDistanceCheckers.checkMinimum3dDistanceConstraintAtTime(posesCurrentTime, 0.5, (timeStep)/1000.0f);
		if (violation.isPresent()) {
			Pose collisionPose = violation.get().getFirstPose();
			CollisionView cv = new CollisionView(this, Point3D.create(collisionPose.x(), collisionPose.y(),  collisionPose.z()));
			collisions.add(cv);
		}
		popMatrix();
		
		popMatrix();
	}

	private int getCurrentTimeStep() {
		int proposedTimeStep = 0, timeStep = 0;
		if (simulationIsActive()) {
			int currentTime = millis();
			proposedTimeStep = currentTime-initialTime+deltaTime;
		} else {
			proposedTimeStep = lastTimeStep;
		}
		
		if (proposedTimeStep/1000.0f - choreo.getChoreographyDuration() < 0.0001) {
			timeStep = proposedTimeStep;
			lastTimeStep = timeStep;
		} else {
			timeStep = lastTimeStep;
		}

		return timeStep;
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
		    zoom += 0.01;
		  } 
		  else if (zoom >= MIN_ZOOM) {
		    zoom -= 0.01; 
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
		
		if (event.getKey() == 'd') {
			droneNameToggle();
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
	 * Activates/deactivates the display of the drone name
	 */
	private void droneNameToggle() {
		if (droneNameActive  == true) {
			droneNameActive = false;
		} else {
			droneNameActive = true;
		}
	}
	
	/**
	 * Activates/deactivates the simulation
	 */
	private void pauseToggle() {
		if (simulationActive == true) {
			simulationActive = false;
			deltaTimeTemp = lastTimeStep;
		} else {
			simulationActive = true;
			initialTime = millis();
			deltaTime = deltaTimeTemp;
		}
	}
	
	private boolean isMouseActive() {
		return mouseActive;
	}
	
	public boolean isTimerActive() {
		return timerActive;
	}
	
	public boolean isDroneNameActive() {
		return droneNameActive ;
	}
	
	private boolean simulationIsActive() {
		return simulationActive;
	}
	/**
	 * Draws timer
	 * @param time in milliseconds
	 */
	public void drawTimer(double time, String msg) {
		pushMatrix();
		rotateX(-PI/2);
		fill(255);
		textSize(72);
		
		int seconds = (int) (time / 1000) % 60 ;
		int minutes = (int) ((time / (1000*60)) % 60);
		int milliseconds = (int) (time % 1000);
		text(String.format("%02d' %02d\" %03d", minutes, seconds, milliseconds), -100.0f, -450.0f, 0.0f);
		text(msg, 400.0f, -450.0f, 0.0f);
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
