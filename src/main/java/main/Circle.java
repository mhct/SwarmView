package main;
import processing.core.PApplet;

/**
 * Visual representation of a drone
 * 
 * @author Mario h.c.t.
 *
 */
public class Circle {
	private final PApplet parent;
	private int SIZE = 20;
	private int colorR, colorG, colorB;
	private int x, y;
	private boolean down, right;
	private int alfa;
	
	public Circle(PApplet parent, int color, int x, int y) {
		this.parent = parent;
		this.colorR = color;
		this.x = x;
		this.y = y;
		this.down = true;
		this.right = true;
		this.alfa = 255;
	}
	
	public void displayNext() {
		if (down) {
			y = y + (int) Math.ceil(parent.random(0,10) * parent.random(1));
		} else {
			y = y - (int) Math.ceil(parent.random(1, 5) * parent.random(1));
		}
		
		if (down && y >= parent.height) {
			down = false;
			fade();
		} else if (!down && y <= 0){
			down = true;
			fade();
		}
		
		if (right) {
			x = x + (int) Math.ceil(parent.random(1,10) * parent.random(1));
		} else {
			x = x - (int) Math.ceil(parent.random(2,13) * parent.random(1));
		}
		
		if (right && x >= parent.width) {
			right = false;
			fade();
		} else if (!right && x <= 0){
			right = true;
			fade();
		}
		
		parent.fill(colorR, colorG, colorB, alfa);
		parent.ellipse(x, y, SIZE, SIZE);
	}
	
	void fade() {
		colorR = (int) parent.random(1, 255);
		colorG = (int) parent.random(1, 255);
		colorB = (int) parent.random(1, 255);
		
		if (SIZE > 0) {
			SIZE = SIZE - 1;
		} else {
			SIZE = 0;
		}
		
		if (alfa > 0) {
			alfa -= 0;
		} else {
			alfa = 0;
		}
	}
}
