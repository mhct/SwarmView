package main;

import processing.core.PApplet;

/**
 * Visual representation of a drone
 * 
 * @author Mario h.c.t.
 *
 */
public class SnowFlake {
	private final PApplet parent;
	private int SIZE = 14;
	private int x, y;
	private boolean down, right;
	private int alfa;
	private float length;
	
	public SnowFlake(PApplet parent, int color, int x, int y) {
		this.parent = parent;
		this.x = x;
		this.y = y;
		this.down = true;
		this.right = true;
		this.alfa = 255;
		this.length = parent.random(7, 12);
	}
	
	public void displayNext() {
		if (down) {
			y = y + (int) Math.ceil(parent.random(0,2) * parent.random(1));
			x = x + (int) Math.ceil(parent.random(-2,2) * parent.random(1));
			
		}
		
		if (down && y >= parent.height-6) {
			down = false;
		} 
		
		
//		fade();
		parent.fill(255, alfa);
		parent.ellipse(x, y, 10, 10);
//		parent.stroke(255);
//		parent.line(x, y, x+length, y+length);
//		parent.line(x+length, y, x, y+length);
//		parent.line(x+length/2.0f, y, x+length/2.0f, y+length);
//		parent.line(x, y+length/2.0f, x+length, y+length/2.0f);
	}
	
	void fade() {

		if (SIZE > 0) {
			SIZE = SIZE - 1;
		} else {
			SIZE = 0;
		}
		
		if (alfa > 0) {
			alfa -= 3;
		} else {
			alfa = 0;
		}
	}
}
