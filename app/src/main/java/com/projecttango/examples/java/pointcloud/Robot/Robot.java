package com.projecttango.examples.java.pointcloud.Robot;

import android.util.Log;

import com.projecttango.examples.java.pointcloud.MainActivity;

import java.util.ArrayList;

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
    private ArrayList<Bucket> bucket;
    private int speed,turning;
    private MainActivity mMainActivity;
    public final String TAG = "ROBOT";

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

    public Robot(MainActivity activity) {
        mMainActivity = activity;
        position = new double[] {0, 0};
        orientation = new double[] {0, 0};
        speed = mMainActivity.DEFAULT_PWM;
        turning = mMainActivity.DEFAULT_PWM;
        bucket = new ArrayList<>();
    }

    public void addBucket(double[] location) {
        bucket.add(new Bucket(location));
        Log.d(TAG, "Bucket sighting: (" + location[0] + ", " + location[1] + ")");
    }

    /*
     * returns the index in ArrayList of the nearest bucket we've sighted.
     * returns -1 if no buckets have been sighted.
     */
    public Bucket nearestBucket() {
        double smallestDistance = (double) Integer.MAX_VALUE;
        int index = -1;
        double distance;
        for(int i = 0; i < bucket.size(); i++) {
            distance = distance(bucket.get(i).location());
            if(distance < smallestDistance) {
                smallestDistance = distance;
                index = i;
            }
         }
        return bucket.get(index);
    }

    public void update() {
        Log.d(TAG,"Update: buckets sighted: " + bucket.size());
        //if we've seen any buckets, go to the closest one
        if(bucket.size() > 0)
            gotoLocation(nearestBucket().location());
        //otherwise, wander around until we have.
        else {
            setSpeed(300);
            setSteering(200);
        }
    }

        /*
         * this is the basic "go to this place" algorithm
         * from the pathfinder, get the next (most time-efficient) target location
         * pass that target location to gotoLocation(), and the robot should make it there.
         * after you've scanned, pass the next pathfinder target location, wrinse, and repeat.
         */

    public void gotoLocation(double[] target) {
        Log.d(TAG,"Going to location: (" + target[0] + ", " + target[1] + ")");
        double angleTolerance = 5;
        //target[0] = x
        //target[1] = y
        double[] targetOrientation = deltaVector(target);
        targetOrientation[0] /= distance(target);
        targetOrientation[1] /= distance(target);

        faceLocation(targetOrientation, angleTolerance);

        if(distance(target) > 0.5) {
            targetOrientation = deltaVector(target);
            targetOrientation[0] /= distance(target);
            targetOrientation[1] /= distance(target);
            setSpeed(300);
            setSteering((int) Math.round((Math.atan2(targetOrientation[1], targetOrientation[0])
                                - Math.atan2(orientation[1], orientation[0]))));
        }
    }

    public void faceLocation(double[] targetOrientation, double tolerance) {
        double tolerance1 = 90;
        double tolerance2 = 45;
        double angle = Math.atan2(targetOrientation[1], targetOrientation[0])
                - Math.atan2(orientation[1], orientation[0]);
        Log.d(TAG, "Delta Angle: " + angle);
        if(Math.abs(angle) <= tolerance1) {
            setSteering(0);
        }
        else if(Math.abs(angle) > tolerance1) {
            setSteering((int) (angle*4)); //sign may need to be flipped
        }
        else if(Math.abs(angle) > tolerance2) {
            setSteering((int) (angle*3)); //sign needs to be same as above
        }
        else if(Math.abs(angle) > tolerance) {
            setSteering((int) (Math.signum(angle)*100));
        }

    }


    /*
     * just in case you need some
     * amphetamines to get through the day
     */
    public int getSpeed() {
        return speed;
    }
    public void setSpeed(int s) {
        Log.d(TAG, "Set Speed: " + s);
        speed = s;
        mMainActivity.set_speed(mMainActivity.DEFAULT_PWM + speed);
    }
    public int getSteering() {
        return turning;
    }
    public void setSteering(int s) {
        Log.d(TAG, "Set Steer: " + s);
        turning = s;
        mMainActivity.set_speed(mMainActivity.DEFAULT_PWM + turning);
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
    public double distance(Double[] target) {
        double[] t = new double[] {target[0], target[1]};
        return distance(t);
    }
}