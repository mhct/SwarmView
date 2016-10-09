package main;

import processing.core.PApplet;

public class Stripe {
	private final PApplet parent;
	private float x;
	private float speed;
	private float w;
	
	Stripe(PApplet parent) {
		this.parent = parent;
		x = 0;
		speed = parent.random(2);
		w = parent.random(10, 30);
	}
	
	
	public void move() {
		x += speed;
		if (x >= parent.width + 20) {
			x = -20;
		}
	}
	
	public void display() {
		parent.fill(123, 100);
		parent.noStroke();
		parent.rect(x, 0, w, parent.height);
	}
	
}
