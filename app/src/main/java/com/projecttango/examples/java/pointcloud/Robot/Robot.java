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
    private ArrayList<Bucket> buckets;
    private int speed,turning;
    private MainActivity mMainActivity;
    public final String TAG = "ROBOT";

    private int wandering_timer;
    private int current_target_index;

        /*
         * position is a 2D vector {x, y}
         * * in your actual code, position should be the tango pose data, which needs to be updated during
         * * the gotoBucketLocation algorithm
         * orientation is a complex number (a + bi) ==> {a, b}
         * * in your actual code, orientation should be the tango pose data, which needs to be updated during
         * * the gotoBucketLocation algorithm
         * you can simplify the quaternion w, x, y, z to the complex number a, b
         * 				I think.
         */

    public Robot(MainActivity activity) {
        mMainActivity = activity;
        position = new double[] {0, 0};
        orientation = new double[] {0, 0};
        speed = mMainActivity.DEFAULT_PWM;
        turning = mMainActivity.DEFAULT_PWM;
        buckets = new ArrayList<>();
        wandering_timer = 0;
    }

    public void onBucketSighting(double distance) {
        double[] predLocation = new double[] {
            position[0] + distance * orientation[0],
                position[1] + distance * orientation[1]
        };
        addBucket(predLocation);
    }
    public void addBucket(double[] location) {
        buckets.add(new Bucket(location));
        Log.d(TAG, "Bucket sighting: (" + location[0] + ", " + location[1] + ")");
    }

    /*
     * returns the index in ArrayList of the nearest buckets we've sighted.
     * returns -1 if no buckets have been sighted.
     */
    public int nearestUnscannedBucketIndex() {
        double smallestDistance = (double) Integer.MAX_VALUE;
        int index = -1;
        double distance;
        for(int i = 0; i < buckets.size(); i++) {
            distance = distance(buckets.get(i).location());
            if(distance < smallestDistance && !buckets.get(i).isScanned()) {
                smallestDistance = distance;
                index = i;
            }
         }
        return index;
    }

    public void updatePose(double[] vector, double[] complex) {
        orientation = complex;
        position = vector;
    }

    public void update() {
        Log.d(TAG,"onUpdate");

        //if we've seen any buckets, go to the closest one
        current_target_index = nearestUnscannedBucketIndex();
        if(buckets.size() > 0 && current_target_index != -1)
            gotoBucketLocation(current_target_index);
        //otherwise, wander around until we have.
        else if(buckets.size()==0){
            setSteering(75);
            setSpeed(wandering_timer++);
            mMainActivity.constrainMotorValues();
        }
    }

        /*
         * this is the basic "go to this place" algorithm
         * from the pathfinder, get the next (most time-efficient) target location
         * pass that target location to gotoBucketLocation(), and the robot should make it there.
         * after you've scanned, pass the next pathfinder target location, wrinse, and repeat.
         */

    public void gotoBucketLocation(int bucketIndex) {
        double[] target = buckets.get(bucketIndex).location();
        Log.d(TAG,"Going to location: (" + target[0] + ", " + target[1] + ")");
        double angleTolerance = 5;
        //target[0] = x
        //target[1] = y
        double[] targetOrientation = deltaVector(target);
        targetOrientation[0] /= distance(target);
        targetOrientation[1] /= distance(target);

        faceLocation(targetOrientation, angleTolerance);

        if(distance(target) > 0.2) {
            targetOrientation = deltaVector(target);
            targetOrientation[0] /= distance(target);
            targetOrientation[1] /= distance(target);
            setSpeed(300);
            setSteering((int) Math.round((Math.atan2(targetOrientation[1], targetOrientation[0])
                                - Math.atan2(orientation[1], orientation[0]))));
        }
        else {
            setSpeed(0);
            setSteering(0);
            mMainActivity.safeSleep(2000);
            mMainActivity.addRajBucket();
            //turn the robot around
            targetOrientation = new double[] {
                    -orientation[0],
                    -orientation[1]
            };
            faceLocation(targetOrientation, 10);
        }
    }

    public void faceLocation(double[] targetOrientation, double tolerance) {
//        double tolerance1 = 90;
//        double tolerance2 = 45;
        double angle = Math.atan2(targetOrientation[1], targetOrientation[0])
                - Math.atan2(orientation[1], orientation[0]);
        Log.d(TAG, "Delta Angle: " + angle);

        if(Math.abs(angle) > tolerance) {
            setSteering((int)Math.round(angle));
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