package io.github.agentwise.swarmview.visualization;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import io.github.agentwise.swarmview.trajectory.control.DroneName;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import io.github.agentwise.swarmview.trajectory.rats.RatsShow;
import processing.core.PApplet;

/**
 * 3D visualization of drone trajectories using ROS
 * 
 * @author Mario h.c.t.
 *
 */
public class ROSView extends MultiDronesUI {

  public static void main(String[] args) {
    PApplet.main(LogsView.class);
  }

  @Override
  protected void initializeVisualization() {
	// Restart the simulated time
	initialTime = millis();
    deltaTime = 0;

    collisions = new ArrayList<>();
    int numberDrones;
	//
    // Associates models, given by a choreography, to a views representing each drone
    //
    drones = new DroneView[1];
	
//	for (int i=0; i<numberDrones; i++) {
	  drones[0] = 
          new TrajectoryDroneView(
        		  	this,
		            new FiniteTrajectory4d() {
        		  		List<Pose> loggedPoses = new ArrayList<>();
        		  		
        		  		void FiniteTrajectory4d() {
        		  			try {
								BufferedReader br = new BufferedReader(new FileReader(new File("/tmp/fievel.log")));
								
								String line = "";
								while ((line = br.readLine()) != "") {
									String[] values = line.split(",");
									double x = Double.valueOf(values[0]); 
									double y = Double.valueOf(values[1]); 
									double z = Double.valueOf(values[2]); 
									double yaw = Double.valueOf(values[3]);
									
									loggedPoses.add(Pose.create(x, y, z, yaw));
									
								}
        		  			} catch (FileNotFoundException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							} catch (IOException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
        		  			
        		  		}
        		  		
						@Override
						public double getTrajectoryDuration() {
							return 100;
						}
						
						@Override
						public Pose getDesiredPosition(double timeInSeconds) {
							return null;
						}
					},
		            DEFAULT_DRONEVIEW_COLORS[0 % DEFAULT_DRONEVIEW_COLORS.length].getRGB(),
		            DEFAULT_TRAIL_LENGTH,
		            "Fievel"
	            );
//	  i++;
//	}
  }
}
