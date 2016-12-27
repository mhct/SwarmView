package io.github.agentwise.swarmview.visualization;

import java.util.ArrayList;

import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.rats.RatsShow;
import processing.core.PApplet;

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
public class RatsView extends MultriDronesUI {

  public static void main(String[] args) {
    PApplet.main(RatsView.class);
  }

  @Override
  protected void initializeVisualization() {
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
          new TrajectoryDroneView(
        		  	this,
		            choreo.getFullTrajectory(drone),
		            DEFAULT_DRONEVIEW_COLORS[i % DEFAULT_DRONEVIEW_COLORS.length].getRGB(),
		            DEFAULT_TRAIL_LENGTH,
		            drone.name()
	            );
	  i++;
	}
  }
}
