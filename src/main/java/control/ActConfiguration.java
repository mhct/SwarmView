package control;

import java.util.List;

import com.google.auto.value.AutoValue;

@AutoValue
public abstract class ActConfiguration {
	public static ActConfiguration create(double duration, List<DronePositionConfiguration> dronePositionConfiguration) {
		return new AutoValue_ActConfiguration(duration, dronePositionConfiguration);
	}
	
	abstract double duration();
	abstract List<DronePositionConfiguration> dronePositionConfiguration();
}