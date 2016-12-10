package io.github.agentwise.swarmview.visualization;

import com.google.common.base.Preconditions;

import io.github.agentwise.swarmview.trajectory.control.dto.Pose;
import processing.core.PApplet;

abstract public class DroneView {
	private final MultiDronesUI canvas;
	private final int BUFFER_SIZE;
	private final Sprite[] previousSprites;
	private int spriteIndex = 0;
	private boolean bufferFull = false;
	private int color;
	private String name;
	
	DroneView(MultiDronesUI canvas, int color, int trailSize, String name) {
		Preconditions.checkNotNull(color);
		Preconditions.checkArgument(trailSize >= 0 && trailSize <= 300);
		
		this.name = name;
		this.canvas = canvas;

		this.color = color;
		this.BUFFER_SIZE = trailSize;
		this.previousSprites = new Sprite[BUFFER_SIZE];
	}
	
	DroneView(MultiDronesUI canvas) {
		this(canvas, 255, 50, "");
	}

	abstract Pose getDesiredPosition(float timeStep);
	
	Pose displayNext(float timeStep) {
		Pose pose = getDesiredPosition(timeStep);
		double x = pose.x();
		double y = pose.y();
		double z = pose.z();
		double yaw = pose.yaw();
		
		Sprite currentSprite = Sprite.create((float)x * 100.0f, -(float)y * 100.0f, (float)z * 100.0f, (float) yaw, color);
		
		previousSprites[spriteIndex] = currentSprite;
		if (spriteIndex + 1 == BUFFER_SIZE) {
			bufferFull = true;
		} 
		spriteIndex = (spriteIndex+1) % BUFFER_SIZE;
		showSprites();
		
		if (canvas.isDroneNameActive()) {
			currentSprite.draw(canvas, 255, name);
		} else {
			currentSprite.draw(canvas, 255);
		}

		return pose;
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
		private float x, y, z, yaw;
		private int color;
		
		private Sprite(float x, float y, float z, float yaw, int color) {
			this.x = x;
			this.y = y;
			this.z = z;
			this.yaw = yaw;
			this.color = color;
		}
		
		public void draw(PApplet canvas, float alfa, String spriteMsg) {
			float xYaw = (float) Math.cos(yaw);
			float yYaw = -1 * (float) Math.sin(yaw);
			
			canvas.pushMatrix();
			canvas.noFill();
			canvas.stroke(color, alfa);
			canvas.translate(x, y, z);
			canvas.line(0, 0, 0,  100*xYaw, 100*yYaw, 0);
			
			canvas.rotateX(-PApplet.PI/2);
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
		
		public static Sprite create(float x, float y, float z, float yaw, int color) {
			return new Sprite(x, y, z, yaw, color);
		}
	}
}
