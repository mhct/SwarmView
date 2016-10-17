package control.dto;

import com.google.auto.value.AutoValue;

@AutoValue
abstract public class Pose {
//	public final float x, y, z, yaw;
	
	Pose() {}
	
//	Pose(float x, float y, float z, float yaw) {
//		this.x = x;
//		this.y = y;
//		this.z = z;
//		this.yaw = yaw;
//	}

	/** @return The X coordinate. */
	  public abstract double x();

	  /** @return The Y coordinate. */
	  public abstract double y();

	  /** @return The Z coordinate. */
	  public abstract double z();

	  /** @return The yaw angle orientation. */
	  public abstract double yaw();

}
