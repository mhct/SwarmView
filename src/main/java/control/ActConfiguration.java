package control;

import java.util.List;

import com.google.auto.value.AutoValue;

@AutoValue
public abstract class ActConfiguration {
	public static ActConfiguration create(String actName, List<DronePositionConfiguration> dronePositionConfiguration) {
		return new AutoValue_ActConfiguration(actName, dronePositionConfiguration);
	}

	public static ActConfiguration create(List<DronePositionConfiguration> dronePositionConfiguration) {
		return new AutoValue_ActConfiguration("", dronePositionConfiguration);
	}
	
	abstract List<DronePositionConfiguration> dronePositionConfiguration();

	abstract String actName();
}