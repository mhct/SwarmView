package control;

import com.google.auto.value.AutoValue;

import control.dto.Pose;

@AutoValue
public abstract class DronePositionConfiguration {

	public static DronePositionConfiguration create(DroneName name, Pose initialPosition, Pose finalPosition) {
		return new AutoValue_DronePositionConfiguration(name, initialPosition, finalPosition);
	}
	
	public static DronePositionConfiguration create(DronePositionConfiguration droneConf) {
		return new AutoValue_DronePositionConfiguration(droneConf.name(), droneConf.initialPosition(), droneConf.finalPosition());
	}

	abstract DroneName name();
	abstract Pose initialPosition();
	abstract Pose finalPosition();

}
