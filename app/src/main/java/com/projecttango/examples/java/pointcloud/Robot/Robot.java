package com.projecttango.examples.java.pointcloud.Robot;

/**
 * Created by rafaelszuminski on 5/3/17.
 */

public class Robot {
    /*
    * Problem:
    * we have position data and a list of locations to accomplish
    * pathfinder deals with following the path
    * we need to translate this algorithm to motor data
    */

    private double[] position;
    private double[] orientation;


        /*
         * position is a 2D vector {x, y}
         * * in your actual code, position should be the tango pose data, which needs to be updated during
         * * the gotoLocation algorithm
         * orientation is a complex number (a + bi) ==> {a, b}
         * * in your actual code, orientation should be the tango pose data, which needs to be updated during
         * * the gotoLocation algorithm
         * you can simplify the quaternion w, x, y, z to the complex number a, b
         * 				I think.
         */

    public Robot() {
        position = new double[] {0, 0};
        orientation = new double[] {0, 0};
    }

        /*
         * this is the basic "go to this place" algorithm
         * from the pathfinder, get the next (most time-efficient) target location
         * pass that target location to gotoLocation(), and the robot should make it there.
         * after you've scanned, pass the next pathfinder target location, wrinse, and repeat.
         */

    public void gotoLocation(double[] target) {
        double[] targetOrientation = deltaVector(target);
        targetOrientation[0] = targetOrientation[0]/distance(target);
        targetOrientation[1] = targetOrientation[1]/distance(target);
        double tolerance = 0.1;
		/*
		 * for double tolerance, I chose 0.1 arbitrarily, but it should be about right.
		 * you'll have to experiment with the robot's turning speed and reaction time
		 * to figure out whether it's high or low. (ask me if this doesn't make sense)
		 */
        do {
			/*
			 * turnLeft();?
			 * turnRight();?
			 * basically, in this loop just do something to rotate the robot until the orientation
			 * condition is met. when it's met, the robot should be facing the target location.
			 * you can make the robot rotate by setting the motors to equal/opposite values
			 * 		-they should be relatively low values so it rotates slowly and it'll be most accurate
			 */
        } while(Math.abs(targetOrientation[1]-orientation[1]) < tolerance
                && Math.abs(targetOrientation[0]-orientation[0]) < tolerance);
        //don't forget to kill the motors so it stops rotating
        while(distance(target) > 0) {
            //drive forward
            //probably want to check every so often that you're still facing the right direction
        }
    }
    /*
     * vector between position and target
     */
    public double[] deltaVector(double[] target) {
        return new double[] {target[0] - position[0], target[1] - position[1]};
    }
    /*
     * distance between position and target
     */
    public double distance(double[] target) {
        return Math.sqrt(Math.pow(deltaVector(target)[0], 2) +Math.pow(deltaVector(target)[1], 2));
    }
}
