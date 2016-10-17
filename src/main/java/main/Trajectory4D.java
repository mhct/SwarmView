package main;

public interface Trajectory4D {
	Pose getDesiredPosition(double timeInSeconds);
}
