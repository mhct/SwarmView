package control;

import com.google.auto.value.AutoValue;

@AutoValue
public abstract class ActConfiguration {
	static ActConfiguration create(Act act, double duration) {
		return new AutoValue_ActConfiguration(act, duration);
	}
	
	abstract Act act();
	abstract double duration();
}