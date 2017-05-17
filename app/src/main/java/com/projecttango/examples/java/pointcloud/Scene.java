/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.projecttango.examples.java.pointcloud;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.MotionEvent;

import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.projecttango.examples.java.pointcloud.Data.FloorPlan;
import com.projecttango.examples.java.pointcloud.Data.PathFinder;
import com.projecttango.examples.java.pointcloud.Data.QuadTree;
import com.projecttango.examples.java.pointcloud.rajawali.FrustumAxes;
import com.projecttango.examples.java.pointcloud.rajawali.Grid;
import com.projecttango.examples.java.pointcloud.rajawali.PointCloud;
import com.projecttango.examples.java.pointcloud.rajawali.TouchViewHandler;

import org.rajawali3d.materials.Material;
import org.rajawali3d.materials.textures.ATexture;
import org.rajawali3d.materials.textures.Texture;
import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector2;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.primitives.Cube;
import org.rajawali3d.primitives.Line3D;
import org.rajawali3d.primitives.Plane;
import org.rajawali3d.renderer.RajawaliRenderer;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

/**
 * Renderer for Point Cloud data.
 */
public class Scene extends RajawaliRenderer {

    private static final float CAMERA_NEAR = 0.01f;
    private static final float CAMERA_FAR = 200f;
    private static final int MAX_NUMBER_OF_POINTS = 60000;

    public boolean qrMarker;


    public static final int QUAD_TREE_START = -60;
    //Size of grid when you walk
    //@TODO figure out what unit
    //About A Meter Good Enough.
    public static final int QUAD_TREE_RANGE = 380;

    private TouchViewHandler mTouchViewHandler;

    // Objects rendered in the scene.
    private FrustumAxes mFrustumAxes;
    private Grid mGrid;
    private PointCloud mPointCloud;

    private com.projecttango.examples.java.pointcloud.TangoUtilClasses.tangoutils.rajawali.Pose startPose;
    private com.projecttango.examples.java.pointcloud.TangoUtilClasses.tangoutils.rajawali.Pose endPose;


    private QuadTree data;

    // Rajawali texture used to render the Tango color camera
    private ATexture mTangoCameraTexture;
    // Keeps track of whether the scene camera has been configured
    private boolean mSceneCameraConfigured;

    private FloorPlan floorPlan;
    private com.projecttango.examples.java.pointcloud.TangoUtilClasses.tangoutils.rajawali.Pose startPoint;
    private com.projecttango.examples.java.pointcloud.TangoUtilClasses.tangoutils.rajawali.Pose endPoint;
    private com.projecttango.examples.java.pointcloud.TangoUtilClasses.tangoutils.rajawali.Pose qrPose;
    private List<Cube> pathCubes = new ArrayList<>();
    private boolean fillPath = false;
    private Material blue;
    private Material orange;
    private boolean renderVirtualObjects;
    private Plane posDot;

    private Vector3 qrBucketObjPos;


    Stack linePoints = new Stack();

    private float[] publicRotation;
    private float[] publicTranslation;
    Cube cube = new Cube(1.0f);

    Stack points = new Stack();
    Line3D line = new Line3D(points, 5, 0x00ff00);
    Material lineMat = new Material();







    public Scene(Context context) {
        super(context);
        mTouchViewHandler = new TouchViewHandler(mContext, getCurrentCamera());
        data = new QuadTree(new Vector2(QUAD_TREE_START, QUAD_TREE_START), QUAD_TREE_RANGE, 8);

    }


    @Override
    protected void initScene() {
        mGrid = new Grid(100, 1, 1, 0xFFCCCCCC);
        mGrid.setPosition(0, -1.3f, 0);

        //getCurrentScene().addChild(mGrid);

        Texture arrowNav = new Texture("Arrow", R.drawable.ic_navigation_arrow);

        Material material = new Material();
        try {
            material.addTexture(arrowNav);
        } catch (ATexture.TextureException e) {
            e.printStackTrace();
        }


        //getCurrentScene().addChild(arrowObj);

        mFrustumAxes = new FrustumAxes(3);
        getCurrentScene().addChild(mFrustumAxes);



        blue = new Material();
        blue.setColor(Color.BLUE);
        orange = new Material();
        orange.setColor(Color.argb(134,255,128,0));

        // Indicate four floats per point since the point cloud data comes
        // in XYZC format.
        floorPlan = new FloorPlan(data);
        getCurrentScene().addChild(floorPlan);
        floorPlan.setVisible(renderVirtualObjects);


        mPointCloud = new PointCloud(MAX_NUMBER_OF_POINTS, 4);
            getCurrentScene().addChild(mPointCloud);
            getCurrentScene().setBackgroundColor(Color.parseColor("#FF212121"));
            getCurrentCamera().setNearPlane(CAMERA_NEAR);
            getCurrentCamera().setFarPlane(CAMERA_FAR);
            getCurrentCamera().setFieldOfView(37.5);


    }



    public void setPointCloudOn(){
        mPointCloud.setColor(Color.WHITE);

    }
    public void setPointCloudOff(){
        mPointCloud.setColor(Color.parseColor("#00ff0000"));
    }







    /**
     * Updates the rendered point cloud. For this, we need the point cloud data and the device pose
     * at the time the cloud data was acquired.
     * NOTE: This needs to be called from the OpenGL rendering thread.
     */

    public void updatePointCloud(TangoPointCloudData pointCloudData, float[] openGlTdepth) {
        mPointCloud.updateCloud(pointCloudData.numPoints, pointCloudData.points);
        Matrix4 openGlTdepthMatrix = new Matrix4(openGlTdepth);
        mPointCloud.setPosition(openGlTdepthMatrix.getTranslation());
        // Conjugating the Quaternion is needed because Rajawali uses left-handed convention.
        mPointCloud.setOrientation(new Quaternion().fromMatrix(openGlTdepthMatrix).conjugate());
        linePoints.add(mPointCloud.getVectorCenterOfPointCloud());
        linePoints.add(mFrustumAxes.getVectorCenterOfFrustum());
    }

    /**
     * Updates our information about the current device pose.
     * NOTE: This needs to be called from the OpenGL rendering thread.
     */
    public void updateCameraPose(TangoPoseData cameraPose) {
        float[] rotation = cameraPose.getRotationAsFloats();
        float[] translation = cameraPose.getTranslationAsFloats();
        addQrMarker();
        Quaternion quaternion = new Quaternion(rotation[3], rotation[0], rotation[1], rotation[2]);
        mFrustumAxes.setPosition(translation[0], translation[1], translation[2]);
        // Conjugating the Quaternion is needed because Rajawali uses left-handed convention for
        // quaternions.
        mFrustumAxes.setOrientation(quaternion.conjugate()); //raf was right
        mTouchViewHandler.updateCamera(new Vector3(translation[0], translation[1], translation[2]),
                quaternion);
        floorPlan.setTrajectoryPosition(new Vector3(translation[0], translation[1], translation[2]));
        publicTranslation = translation;
        publicRotation = rotation;


    }

    @Override
    public void onOffsetsChanged(float v, float v1, float v2, float v3, int i, int i1) {
        Log.d("Offset:","Changed");
    }

    @Override
    public void onTouchEvent(MotionEvent motionEvent) {
        mTouchViewHandler.onTouchEvent(motionEvent);
    }

    @Override
    protected void onRender(long ellapsedRealtime, double deltaTime) {
        super.onRender(ellapsedRealtime, deltaTime);
        // add routing cubes to scene graph if available
        if (fillPath) {
            Log.d("Scene onRender","in If statment for fillPath");
            for (Cube pathCube : pathCubes) {
                getCurrentScene().removeChild(pathCube);
                Log.d("Scene onRender","removed all existing path cubes");

            }
            pathCubes.clear();
            PathFinder finder = new PathFinder(floorPlan.getData());
            try {
                Log.d("Scene onRender","in try statment");
                List<Vector2> path = finder.findPathBetween(startPoint.getPosition(), endPoint.getPosition());
                for (Vector2 vector2 : path) {
                    cube.setMaterial(blue);
                    cube.setPosition(new Vector3(vector2.getX(), -1.2, vector2.getY()));
                    getCurrentScene().addChild(cube);
                    pathCubes.add(cube);
                }
            } catch (Exception e) {
                Log.e("Scene:OnRender", "onRender: " + e.getMessage(), e);
            } finally {
                fillPath = false;
            }
        }
    }

    public void setStartPoint(TangoPoseData currentPose) {
        float[] rotation = currentPose.getRotationAsFloats();
        float[] translation = currentPose.getTranslationAsFloats();
        floorPlan.addPoint(new Vector3(translation[0], translation[1], translation[2]));
        if (startPoint != null && endPoint != null) {
            fillPath = true;
        }
    }

    public void setEndPoint(TangoPoseData currentPose) {
        float[] rotation = currentPose.getRotationAsFloats();
        float[] translation = currentPose.getTranslationAsFloats();
        floorPlan.addPoint(new Vector3(translation[0], translation[1], translation[2]));
        if (startPoint != null && endPoint != null) {
            fillPath = true;
        }
    }

    public void addQrMarker(){
        if(qrMarker == true){
            Log.d("QRMarker","Is True");
            addQRBucketObject();
            // Set the pos of the cube at the last pos the Furstrum was at.
            qrMarker = false;
        }else{
            Log.d("QRMarker","Still False");
        }
    }

    public void addQRBucketObject(){
        Plane bucketQR = new Plane(0.7f,0.7f,1,1,1);
        bucketQR.setMaterial(orange);
        bucketQR.setPosition(new Vector3(publicTranslation[0],-1.39,publicTranslation[2]));
        bucketQR.setRotY(90);
        bucketQR.setRotZ(90);
        qrBucketObjPos = bucketQR.getPosition();
        points.add(qrBucketObjPos);
        getCurrentScene().addChild(bucketQR);


    }

    public void drawLineBtwnBuckets(){
        lineMat.setColor(Color.RED);
        line.setMaterial(lineMat);
        getCurrentScene().addChild(line);
    }

    public void removeLineBtwnBuckets(){
        getCurrentScene().removeChild(line);
    }

    public void setTrueMarker(){
        qrMarker = true;
    }

    public void setFirstPersonView() {
        mTouchViewHandler.setFirstPersonView();
    }

    public void setTopDownView() {
        mTouchViewHandler.setTopDownView();
    }

    public void renderVirtualObjects(boolean renderObjects) {
        renderVirtualObjects = renderObjects;
        if (this.floorPlan != null)
            this.floorPlan.setVisible(renderObjects);
    }

    public void setThirdPersonView() {
        mTouchViewHandler.setThirdPersonView();
    }

    public QuadTree getFloorPlanData() {
        return data;
    }

    public QuadTree getData() {
        return data;
    }

}
