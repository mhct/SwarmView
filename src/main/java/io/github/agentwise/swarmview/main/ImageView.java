package io.github.agentwise.main;

import processing.core.PApplet;
import processing.core.PImage;

public class ImageView extends PApplet {
	int displayDimensionX = 800;
	int displayDimensionY = 600;

	PImage img;
	
    @Override
    public void settings() {
    	fullScreen();
        size(displayDimensionX, displayDimensionY, P3D);
    }

    @Override
    public void setup() {
        fill(255);
        img = loadImage("http://www.lower-my-energybill.com/image-files/181x197xwindow-efficiency.jpg.pagespeed.ic.xNoxyGEWWR.jpg", "jpg");
    }

	@Override
	public void draw() {
		background(0);
		image(img, 0, 0);
	}
	
	public static void main(String[] args) {
        PApplet.main(ImageView.class);
    }

}
