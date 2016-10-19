package control.dto;

import com.google.auto.value.AutoValue;

@AutoValue
abstract public class Pose {
	
	public static AutoValue_Pose create(double x, double y, double z, double yaw) {
		return new AutoValue_Pose(x, y, z, yaw);
	}

	  /** @return The X coordinate. */
	  public abstract double x();

	  /** @return The Y coordinate. */
	  public abstract double y();

	  /** @return The Z coordinate. */
	  public abstract double z();

	  /** @return The yaw angle orientation. */
	  public abstract double yaw();

}
