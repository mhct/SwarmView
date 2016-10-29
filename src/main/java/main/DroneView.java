package main;

import com.google.common.base.Preconditions;

import control.FiniteTrajectory4d;
import control.dto.Pose;
import processing.core.PApplet;

public class DroneView {
	private final PApplet canvas;
	private final FiniteTrajectory4d trajectory;
	private final int BUFFER_SIZE;
	private final Sprite[] previousSprites;
	private int spriteIndex = 0;
	private boolean bufferFull = false;
	private int color;
	private String name;
	
	DroneView(RatsView canvas, FiniteTrajectory4d trajectory, int color, int trailSize, String name) {
		Preconditions.checkNotNull(color);
		Preconditions.checkArgument(trailSize >= 0 && trailSize <= 300);
		
		this.canvas = canvas;
		this.trajectory = trajectory;
		this.color = color;
		this.BUFFER_SIZE = trailSize;
		this.name = name;
		this.previousSprites = new Sprite[BUFFER_SIZE];
	}
	
	DroneView(PApplet canvas, FiniteTrajectory4d trajectory) {
		this.canvas = canvas;
		this.trajectory = trajectory;
		this.color = 255;
		this.BUFFER_SIZE = 50;
		this.previousSprites = new Sprite[BUFFER_SIZE];
	}
	
	void displayNext(float timeStep) {
		Pose pose = trajectory.getDesiredPosition(timeStep);
		double x = pose.x();
		double y = pose.y();
		double z = pose.z();
		Sprite currentSprite = Sprite.create((float)x * 100.0f, (float)y * 100.0f, (float)z * 100.0f, color);
		
		previousSprites[spriteIndex] = currentSprite;
		if (spriteIndex + 1 == BUFFER_SIZE) {
			bufferFull = true;
		} 
		spriteIndex = (spriteIndex+1) % BUFFER_SIZE;
		showSprites();
		currentSprite.draw(canvas, 255, name);

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
		private float x, y, z;
		private int color;
		
		private Sprite(float x, float y, float z, int color) {
			this.x = x;
			this.y = y;
			this.z = z;
			this.color = color;
		}
		
		public void draw(PApplet canvas, float alfa, String spriteMsg) {
			canvas.pushMatrix();
			canvas.noFill();
			canvas.stroke(color, alfa);
			canvas.translate(x,  y, z);
			canvas.rotateX(-canvas.PI/2);
			if (!"".equals(spriteMsg)) {
				canvas.textSize(26);
				canvas.text(spriteMsg, -20.0f, -20.0f, -10.0f);
			}
			canvas.sphereDetail(12);
			canvas.sphere(20);
			canvas.stroke(255);
			canvas.popMatrix();
		}
		
		public void draw(PApplet canvas, float alfa) {
			draw(canvas, alfa, "");
		}
		
		public static Sprite create(float x, float y, float z, int color) {
			return new Sprite(x, y, z, color);
		}
	}
}
