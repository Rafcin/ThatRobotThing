package com.projecttango.examples.java.pointcloud.Robot;

/**
 * Created by rafaelszuminski on 5/17/17.
 */

class Bucket {
    private double[] location;
    private boolean scanned;
    public Bucket(double[] loc) {
        location = loc;
        scanned = false;
    }
    public double[] location() {
        return location;
    }
    public void scan() {
        scanned = true;
    }
    public boolean isScanned() {
        return scanned;
    }
}
