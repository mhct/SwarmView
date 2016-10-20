## Instalation
Configure IDE to use autovalue
- http://www.codeaffine.com/2014/03/04/using-the-autovalue-code-generator-in-eclipse/

## Using the View
The view has 4 key-bindings defined:
- **z** activates/deactivates the mouse
- **t** displays/hides the simulation time
- **p** pauses the simulation view. Note that the simulation time continues advancing in the background. Imagine this option as a snapshot in time.
- **r** restarts the simulation

## Programming interface
New **Drone** objects can be created by specifying 
- Drone(PApplet canvas, FiniteTrajectory4d trajectory)
- Drone(PApplet canvas, FiniteTrajectory4d trajectory, int color, int trailSize)

