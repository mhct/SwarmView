package main;

import processing.core.PApplet;

import java.lang.Override;

public class SnowFlakesAnim extends PApplet {
	private int numberFlakes=100, extraFlakes=0;
	SnowFlake[] snowFlakes;
	
	public static void main(String[] args) {
		PApplet.main(SnowFlakesAnim.class);
	}
	
	@Override
	public void settings() {
		fullScreen();
	}
	
	@Override
	public void setup() {
		fill(255, 120, 134);
		numberFlakes += extraFlakes;
		snowFlakes = new SnowFlake[numberFlakes + extraFlakes];
		for (int i=0; i<numberFlakes; i++) {
			snowFlakes[i] = new SnowFlake(this, 255, (int) Math.ceil(random(0, width)), (int) Math.ceil(random(0, height)));
		}
	}
	
	@Override
	public void draw() {
		background(0);
		if (mousePressed) {
			extraFlakes++;
			setup();
		}
		
		for (int i=0; i<numberFlakes; i++) {
			snowFlakes[i].displayNext();
		}
	}

}
