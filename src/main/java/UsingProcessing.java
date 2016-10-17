package main;

import processing.core.PApplet;

import java.awt.Point;
import java.lang.Override;

public class UsingProcessing extends PApplet {
	private final int NUMBER_STRIPES = 100;
//	Stripe[] stripes = new Stripe[NUMBER_STRIPES];
	Circle[] circles = new Circle[NUMBER_STRIPES];
	
	public static void main(String[] args) {
		PApplet.main(UsingProcessing.class);
	}
	
	@Override
	public void settings() {
		fullScreen();
//		size(400, 400);
	}
	
	@Override
	public void setup() {
		fill(255, 120, 134);
		
		for (int i=0; i<NUMBER_STRIPES; i++) {
//			stripes[i] = new Stripe(this);
			circles[i] = new Circle(this, 255, (int) Math.ceil(random(0, width)), (int) Math.ceil(random(0, height)));
		}
		
	}
	
	@Override
	public void draw() {
//		Point loc = frame.getLocation();
		background(0);
		if (mousePressed) {
			setup();
		}
		
		for (int i=0; i<NUMBER_STRIPES; i++) {
			circles[i].displayNext();
		}
	}

}
