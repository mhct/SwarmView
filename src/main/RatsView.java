package main;

import applications.trajectory.CircleTrajectory4D;
import applications.trajectory.geom.point.Point3D;
import control.Trajectory4d;
import processing.core.PApplet;

public class RatsView extends PApplet {
	private final int NUMBER_DRONES = 1;
	Drone[] drones = new Drone[NUMBER_DRONES];
	private float timeStep;
	
	public static void main(String[] args) {
		PApplet.main(RatsView.class);
	}
	
	@Override
	public void settings() {
		fullScreen();
//		size(400, 400);
	}
	
	@Override
	public void setup() {
		fill(255);
		timeStep = 0;
		
		Trajectory4d trajectory = CircleTrajectory4D.builder()
				.setLocation(Point3D.create(width/2, height/2, 100))
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
		
		for (int i=0; i<NUMBER_DRONES; i++) {
			drones[i].displayNext(timeStep);
		}
		timeStep = timeStep + 1.0f;
	}

}
