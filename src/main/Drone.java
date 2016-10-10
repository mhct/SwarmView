package main;

import control.Trajectory4d;
import processing.core.PApplet;

public class Drone {
	private final PApplet canvas;
	private final Trajectory4d trajectory;
	private final int BUFFER_SIZE = 50;
	private Sprite[] previousSprites = new Sprite[BUFFER_SIZE];
	private int spriteIndex = 0;
	private boolean bufferFull = false;
	
	Drone(PApplet canvas, Trajectory4d trajectory) {
		this.canvas = canvas;
		this.trajectory = trajectory;
	}
	
	void displayNext(float timeStep) {
		double x = trajectory.getDesiredPositionX(timeStep);
		double y = trajectory.getDesiredPositionY(timeStep);
		double z = trajectory.getDesiredPositionZ(timeStep);
//		double yaw = trajectory.getDesiredAngleZ(timeStep);
		Sprite currentSprite = Sprite.create((float)x, (float)y);
//		Sprite currentSprite = Sprite.create(canvas.mouseX, canvas.mouseY);
		
		previousSprites[spriteIndex] = currentSprite;
		if (spriteIndex + 1 == BUFFER_SIZE) {
			bufferFull = true;
		} 
		spriteIndex = (spriteIndex+1) % BUFFER_SIZE;
		showSprites();
		currentSprite.draw(canvas, 255);

	}
	
	
	void showSprites() {
		float alfaDelta;
		alfaDelta = 255.0f/(float)BUFFER_SIZE;
		
		
		if (bufferFull) {
			for (int i=spriteIndex; i<BUFFER_SIZE; i++) {
				previousSprites[i].draw(canvas, (i-spriteIndex) * alfaDelta);
			}
			for (int i=0; i<spriteIndex; i++) {
				previousSprites[i].draw(canvas, (BUFFER_SIZE-spriteIndex+i) * alfaDelta);
			}
		} else if (spriteIndex > 1) {
			for (int i=0; i<spriteIndex; i++) {
				previousSprites[i].draw(canvas, i * alfaDelta);
			}
		}
	}
	
	static class Sprite {
		private float x, y;
		
		private Sprite(float x, float y) {
			this.x = x;
			this.y = y;
		}
		
		public void draw(PApplet canvas, float alfa) {
			canvas.noStroke();
			canvas.fill(255,  alfa);
			canvas.pushMatrix();
			canvas.noFill();
			canvas.stroke(255, alfa);
			canvas.translate(x,  y, 0);
//			canvas.ellipse(x, y, 10, 10);
			canvas.sphereDetail(20);
			canvas.sphere(10);
			canvas.popMatrix();
		}
		
		public static Sprite create(float x, float y) {
			return new Sprite(x, y);
		}
	}
}
