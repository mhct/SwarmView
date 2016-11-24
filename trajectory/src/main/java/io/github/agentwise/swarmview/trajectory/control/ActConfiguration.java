package io.github.agentwise.swarmview.trajectory.control;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.google.auto.value.AutoValue;

import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

@AutoValue
public abstract class ActConfiguration {
	public static ActConfiguration create(String actName, List<DronePositionConfiguration> dronePositionConfiguration) {
		return new AutoValue_ActConfiguration(actName, dronePositionConfiguration);
	}

	public static ActConfiguration create(List<DronePositionConfiguration> dronePositionConfiguration) {
		return new AutoValue_ActConfiguration("", dronePositionConfiguration);
	}
	
	public static ActConfiguration createFromInitialFinalPositions(Map<DroneName,Pose> initialPositions, Map<DroneName,Pose> finalPositions) {

		List<DronePositionConfiguration> positions = new ArrayList<>();

		for (DroneName drone : DroneName.values()) {
			positions.add(DronePositionConfiguration.create(drone, initialPositions.get(drone),  finalPositions.get(drone)));
		}
		ActConfiguration configuration = ActConfiguration.create(positions);

		return configuration;
	}
	
	public abstract String actName();

	public abstract List<DronePositionConfiguration> dronePositionConfiguration();
	
	public Map<DroneName, Pose> initialPositionConfiguration() {
		Map<DroneName, Pose> initialPositions = new LinkedHashMap<>();
		for (DronePositionConfiguration c: dronePositionConfiguration()) {
			initialPositions.put(c.name(), c.initialPosition());
		}
		
		return initialPositions;
	}
	
	public Map<DroneName, Pose> finalPositionConfiguration() {
		Map<DroneName, Pose> finalPositions = new LinkedHashMap<>();
		for (DronePositionConfiguration c: dronePositionConfiguration()) {
			finalPositions.put(c.name(), c.finalPosition());
		}
		
		return finalPositions;
	}
	
	public int numberDrones() {
		return dronePositionConfiguration().size();
	}
	
}