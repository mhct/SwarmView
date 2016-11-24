package io.github.agentwise.swarmview.visualization;

import processing.core.PConstants;

/** Represents the Stage and Flying Area**/
public class StageView {
	  private int stageHeight; // Height, Depth, Width, given in centimeters
	  private int stageDepth;
	  private int stageWidth;
	  private int stageDrawingInclinedPlaneX;
	  private int stageDrawingInclinedPlaneZ;

	private RatsView canvas;

	public StageView(RatsView canvas, int stageWidth, int stageDepth, int stageHeight, int stageDrawingInclinedPlaneX, int stageDrawingInclinedPlaneZ) {
		this.canvas = canvas;
		this.stageWidth = stageWidth;
		this.stageDepth = stageDepth;
		this.stageHeight = stageHeight;
		this.stageDrawingInclinedPlaneX = stageDrawingInclinedPlaneX;
		this.stageDrawingInclinedPlaneZ = stageDrawingInclinedPlaneZ;
	}
	
	public void draw() {
		drawFlyingZone();
		drawStage();
	}
	
	  /** Draws the stage */
  private void drawStage() {
    canvas.pushMatrix();
    canvas.scale(700, 700, 700);

    // Room floor
    canvas.noStroke();
    canvas.beginShape(PConstants.QUADS);
    canvas.fill(255, 255, 0, 100);
    canvas.vertex(-1, -1, 0);
    canvas.vertex(1, -1, 0);
    canvas.vertex(1, 1, 0);
    canvas.vertex(-1, 1, 0);
    canvas.endShape();

    // Room back (x,z) plane
    canvas.noStroke();
    canvas.beginShape(PConstants.TRIANGLE);
    canvas.fill(255, 0, 255, 100);
    canvas.vertex(-1, -1, 0);
    canvas.vertex(0, -1, 1);
    canvas.vertex(1, -1, 0);
    canvas.endShape();

    canvas.popMatrix();
  }

  private void drawFlyingZone() {
	  canvas.pushMatrix();
	  canvas.translate(-stageWidth/2.0f, -stageDepth/2.0f, 0.0f);
	  canvas.strokeWeight(3.0f);
	  canvas.text("Origin", 0.0f, 0.0f, 0.0f);
	    
	    //Back plane
	  canvas.beginShape(PConstants.QUAD);
	  canvas.noFill();
	  canvas.vertex(0, 0, 0);
	  canvas.vertex(stageWidth, 0, 0);
	  canvas.vertex(stageWidth, 0, stageHeight);
	  canvas.vertex(0, 0, stageHeight);
	  canvas.endShape();

	    //Left plane (looking from audience)
	  canvas.beginShape(PConstants.POLYGON);
	  canvas.noFill();
	  canvas.vertex(0, 0, 0);
	  canvas.vertex(0, stageDepth, 0);
	  canvas.vertex(0, stageDepth, stageDrawingInclinedPlaneZ);
	  canvas.vertex(0, stageDrawingInclinedPlaneX, stageHeight);
	  canvas.vertex(0, 0, stageHeight);
	  canvas.endShape();

	    //Right plane (looking from audience)
	  canvas.beginShape(PConstants.POLYGON);
	  canvas.noFill();
	  canvas.vertex(stageWidth, 0, 0);
	  canvas.vertex(stageWidth, stageDepth, 0);
	  canvas.vertex(stageWidth, stageDepth, stageDrawingInclinedPlaneZ);
	  canvas.vertex(stageWidth, stageDrawingInclinedPlaneX, stageHeight);
	  canvas.vertex(stageWidth, 0, stageHeight);
	  canvas.endShape();

	    //Inclined front plane
	  canvas.beginShape(PConstants.QUAD);
	  canvas.noFill();
	  canvas.vertex(0, stageDepth, stageDrawingInclinedPlaneZ);
	  canvas.vertex(stageWidth, stageDepth, stageDrawingInclinedPlaneZ);
	  canvas.vertex(stageWidth, stageDrawingInclinedPlaneX, stageHeight);
	  canvas.vertex(0, stageDrawingInclinedPlaneX, stageHeight);
	  canvas.endShape();

	    // Front plane, below inclined plane
	  canvas.beginShape(PConstants.QUAD);
	  canvas.noFill();
	  canvas.vertex(0, stageDepth, 0);
	  canvas.vertex(stageWidth, stageDepth, 0);
	  canvas.vertex(stageWidth, stageDepth, stageDrawingInclinedPlaneZ);
	  canvas.vertex(0, stageDepth, stageDrawingInclinedPlaneZ);
	  canvas.endShape();
	  canvas.popMatrix();

	    //Draws markers on each meter on the ground (along Y axis)
	    int drawingIntervalInCm = 100;
	    for (int i=0; i<=stageDepth; i=i+drawingIntervalInCm) {
	    	canvas.pushMatrix();
	    	canvas.translate(-stageWidth/2, -stageDepth/2, 0);
	    	canvas.translate(0, i, 2);
	    	canvas.box(4);

	    	canvas.translate(stageWidth, 0, 0);
	    	canvas.box(4);
	    	canvas.popMatrix();
	    }

	    //Draws markers on each meter on the ground (along X axis)
	    for (int i=0; i<=stageWidth; i=i+drawingIntervalInCm) {
	    	canvas.pushMatrix();
	    	canvas.translate(-stageWidth/2, -stageDepth/2, 0);
	    	canvas.translate(i, 0, 2);
	    	canvas.box(4);
	    	
	    	canvas.translate(0, stageDepth, 0);
	    	canvas.box(4);
	    	canvas.popMatrix();
	    }
  }
}
