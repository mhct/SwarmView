package io.github.agentwise.swarmview.visualization;

import java.awt.Color;
import java.awt.MouseInfo;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.checkers.OfflineMinimumDistanceCheckers;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.control.ChoreographyView;
import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.rats.RatsShow;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.event.KeyEvent;
import processing.event.MouseEvent;

/**
 * 3D visualization of drone trajectories.
 * 
 * The frame of reference used in the view is defined as below, following the convention used by Processing @see <a href="http://processing.org">http://processing.org</a>
 * 
 * --> X  
 * |           Z points at the same direction as the vector Y x X  (cross product of Y and X) 
 * \/
 * Y
 * 
 * @author Mario h.c.t.
 *
 */
public class RatsView extends PApplet {
  private static final int STAGE_DEPTH = 530;
  private static final int STAGE_WIDTH = 700;
  private static final int STAGE_HEIGHT = 350; // Height, Depth, Width, given in centimeters
  private static final int STAGE_DRAWING_INCLINED_PLANE_X = 355;
  private static final int STAGE_DRAWING_INCLINED_PLANE_Z = 100;

  private static final float MAX_ZOOM = 4.0f;
  private static final float MIN_ZOOM = 0.3f;
  private static final Color[] DEFAULT_DRONEVIEW_COLORS = {Color.CYAN, Color.YELLOW, Color.PINK, Color.GREEN, Color.BLUE};
  private static final int DEFAULT_TRAIL_LENGTH = 10;

  private final int displayDimensionX = 1024;
  private final int displayDimensionY = 800;
  private int rotzfactor = 0;
  private float zoom = 1.0f;
  private boolean mouseActive = true;
  private boolean timerActive = true;
  private boolean droneNameActive = true;
  private boolean simulationActive = false;

  private int lastMouseX;
  private int lastMouseY;
  private float lastZoom;
  private float rotz;

  private int lastTimeStep;
  private int initialTime = 0;
  private int deltaTime = 0;
  private int deltaTimeTemp;

  private ChoreographyView choreo;
  private StageView stage;
  private DroneView[] drones;
  private List<CollisionView> collisions;

  public static void main(String[] args) {
    PApplet.main(RatsView.class);
  }

  @Override
  public void settings() {
    size(displayDimensionX, displayDimensionY, PConstants.P3D);
  }

  @Override
  public void setup() {
    fill(255);
    initializeVisualization();
    stage = new StageView(this, STAGE_WIDTH, STAGE_DEPTH, STAGE_HEIGHT, STAGE_DRAWING_INCLINED_PLANE_X, STAGE_DRAWING_INCLINED_PLANE_Z);
  }

  private void initializeVisualization() {
	choreo = RatsShow.createChoreography();
    
	// Restart the simulated time
	initialTime = millis();
    deltaTime = 0;

    collisions = new ArrayList<>();
    //
    // Associates models, given by a choreography, to a views representing each drone
    //
    drones = new DroneView[choreo.getNumberDrones()];
	
    int i=0;
	for (DroneName drone: choreo.getDroneNames()) {
	  drones[i] = 
          new DroneView(
        		  	this,
		            choreo.getFullTrajectory(drone),
		            DEFAULT_DRONEVIEW_COLORS[i % DEFAULT_DRONEVIEW_COLORS.length].getRGB(),
		            DEFAULT_TRAIL_LENGTH,
		            drone.name()
	            );
	  i++;
	}
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

    stage.draw();
	pushMatrix();
	translate(-STAGE_WIDTH/2.0f, -STAGE_DEPTH/2.0f, 0.0f);
	
    int timeStep = getCurrentTimeStep();
    if (isTimerActive()) {
      drawTimer(timeStep, choreo.getCurrentActName(timeStep / 1000.0f));
    }
    List<Pose> posesCurrentTime = new ArrayList<Pose>();
    for (int i = 0; i < drones.length; i++) {
      Pose pose = drones[i].displayNext((timeStep) / 1000.0f);
      posesCurrentTime.add(pose);
    }

    // show previous collisions
    for (CollisionView coll : collisions) {
      coll.displayNext();
    }
    
    //
    // calculates current collisions
    //
    Optional<OfflineMinimumDistanceCheckers.Violation> violation =
        OfflineMinimumDistanceCheckers.checkMinimum3dDistanceConstraintAtTime(
            posesCurrentTime, 0.5, (timeStep) / 1000.0f);
    if (violation.isPresent()) {
      Pose collisionPose = violation.get().getFirstPose();
      CollisionView cv =
          new CollisionView(
              this, Point3D.create(collisionPose.x(), collisionPose.y(), collisionPose.z()));
      collisions.add(cv);
    }
    popMatrix();

    popMatrix();
  }

  private int getCurrentTimeStep() {
    int proposedTimeStep = 0, timeStep = 0;
    if (simulationIsActive()) {
      int currentTime = millis();
      proposedTimeStep = currentTime - initialTime + deltaTime;
    } else {
      proposedTimeStep = lastTimeStep;
    }

    if (proposedTimeStep / 1000.0f - choreo.getChoreographyDuration() < 0.0001) {
      timeStep = proposedTimeStep;
      lastTimeStep = timeStep;
    } else {
      timeStep = lastTimeStep;
    }

    return timeStep;
  }

  /** Positions the Scene view according to the last mouse movement */
  private void positionView(float x, float y, float zoom) {
    zoom = PApplet.constrain(zoom, MIN_ZOOM, MAX_ZOOM);
    scale(zoom);

    /** Rotation of the screen */
    float rotx = (2 * x / (float) displayDimensionX) * -2 * PConstants.PI + PConstants.PI;
    float roty = (2 * y / (float) displayDimensionY) * -2 * PConstants.PI - PConstants.PI;
    rotz = rotzfactor * PConstants.PI / 36;

    pushMatrix();
    translate(width / (2 * zoom), height / (2 * zoom), rotz / zoom);
    rotateX(roty);
    rotateZ(rotx);
  }

  public void mouseWheel(MouseEvent event) {
    if (event.getCount() >= 0 && zoom <= MAX_ZOOM) {
      zoom += 0.01;
    } else if (zoom >= MIN_ZOOM) {
      zoom -= 0.01;
    }
  }

  /** Handles user input via the keyboard */
  @Override
  public void keyPressed(KeyEvent event) {
    switch(event.getKey()) {
    case 'z': mouseToggle();
    		  break;
    case 't': timerToggle();
    		  break;
    case ' ': pauseToggle();
    	      break;
    case 'd': droneNameToggle();
              break;
    case 'r': initializeVisualization();
    		  break;
    case '.': forwardTime(); // RIGHT ARROW
    		  break;
    case ',': backwardTime(); // RIGHT ARROW
    		  break;
    }
    
  }

  /** Activates or deactivates listening to mouse events */
  private void mouseToggle() {
    if (mouseActive == true) {
      mouseActive = false;
    } else {
      mouseActive = true;
    }
  }

  /** Activates/deactivates the timer */
  private void timerToggle() {
    if (timerActive == true) {
      timerActive = false;
    } else {
      timerActive = true;
    }
  }

  /** Activates/deactivates the display of the drone name */
  private void droneNameToggle() {
    if (droneNameActive == true) {
      droneNameActive = false;
    } else {
      droneNameActive = true;
    }
  }

  /** Activates/deactivates the display of the drone name */
  private void forwardTime() {
	  initialTime = millis();
      deltaTime += 5000;
  }

  private void backwardTime() {
	  initialTime = millis();
	  if (deltaTime <= 10000) {
		  deltaTime = 0;
	  } else {
		  deltaTime -= 5000;
	  }
  }

  /** Activates/deactivates the simulation */
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
    return droneNameActive;
  }

  private boolean simulationIsActive() {
    return simulationActive;
  }
  /**
   * Draws timer
   *
   * @param time in milliseconds
   */
  public void drawTimer(double time, String msg) {
    pushMatrix();
    rotateX(-PConstants.PI / 2);
    fill(255);
    textSize(72);

    int seconds = (int) (time / 1000) % 60;
    int minutes = (int) ((time / (1000 * 60)) % 60);
    int milliseconds = (int) (time % 1000);
    text(
        String.format("%02d' %02d\" %03d", minutes, seconds, milliseconds), -100.0f, -450.0f, 0.0f);
    text(msg, 400.0f, -450.0f, 0.0f);
    popMatrix();
  }
}
