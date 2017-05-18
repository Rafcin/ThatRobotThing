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

import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.Camera;
import android.hardware.display.DisplayManager;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.speech.tts.TextToSpeech;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.Display;
import android.view.InputDevice;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.afollestad.materialdialogs.DialogAction;
import com.afollestad.materialdialogs.GravityEnum;
import com.afollestad.materialdialogs.MaterialDialog;
import com.afollestad.materialdialogs.Theme;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoTextureCameraPreview;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.projecttango.examples.java.pointcloud.Data.MapView;
import com.projecttango.examples.java.pointcloud.MapViewTools.MapInfo;
import com.projecttango.examples.java.pointcloud.OpenCV.ColorBlobDetector;
import com.projecttango.examples.java.pointcloud.Robot.Robot;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;
import com.projecttango.tangosupport.ux.TangoUx;
import com.projecttango.tangosupport.ux.UxExceptionEvent;
import com.projecttango.tangosupport.ux.UxExceptionEventListener;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Locale;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the {@link Tango}
 * service and propagation of Tango point cloud data to OpenGL and Layout views. OpenGL rendering
 * logic is delegated to the {@link Scene} class.
 */
public class MainActivity extends IOIOActivity{
    private static final String TAG = MainActivity.class.getSimpleName();

    private static final String UX_EXCEPTION_EVENT_DETECTED = "Exception Detected: ";
    private static final String UX_EXCEPTION_EVENT_RESOLVED = "Exception Resolved: ";

    private static final int SECS_TO_MILLISECS = 1000;
    private static final DecimalFormat FORMAT_THREE_DECIMAL = new DecimalFormat("0.000");
    private static final double UPDATE_INTERVAL_MS = 100.0;

    private Tango mTango;
    private TangoConfig mConfig;
    private TangoUx mTangoUx;

    private TangoPointCloudManager mPointCloudManager;
    private Scene mRenderer;
    private RajawaliSurfaceView mSurfaceView;
    private TextView mPointCountTextView;

    private TextView mAverageZTextView;
    private double mPointCloudPreviousTimeStamp;

    private boolean mIsConnected = false;

    private double mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;

    private int mDisplayRotation = 0;

    private MapInfo mapInfo;

    private static final int GRID_SIZE = 100;

    public MapView mMapView;


    public TangoPoseData mapPos;

    private int primaryDark;
    private int primaryColor;
    private int primaryTextColor;

    private TextToSpeech textToSpeech;

    private TangoTextureCameraPreview tangoCameraPreview;

    private int mDepthCameraToDisplayRotation = 0;

    private static final int PERMISSIONS_REQUEST = 1;

    private static final String PERMISSION_CAMERA = Manifest.permission.CAMERA;
    private static final String PERMISSION_STORAGE = Manifest.permission.WRITE_EXTERNAL_STORAGE;

    final Bitmap[] bm = new Bitmap[1];
    Bitmap bmC = Bitmap.createBitmap(224, 224, Bitmap.Config.ARGB_8888);
    int frameCount = 0;

    //IOIO
    public PwmOutput pwm_speed_output, pwm_steering_output, pwm_pan_output, pwm_tilt_output;
    int pwm_pan, pwm_tilt, pwm_speed, pwm_steering;
    private AnalogInput sonar1,sonar2,sonar3;
    int sonarPulseCounter;
    private DigitalOutput sonar_pulse;
    int sonar1_reading, sonar2_reading, sonar3_reading;
    public final int DEFAULT_PWM = 1500, MAX_PWM = 2000, MIN_PWM = 1000;

    public Mat tmp;
    private ColorBlobDetector mDetector;
    private Scalar mBlobColorRgba;
    private Mat mSpectrum;
    private Scalar CONTOUR_COLOR;

    private SeekBar mMotorbar;
    private int motorSliderVal;

    private Robot mRobot;
    private boolean isAuto;
    //TODO: isAuto toggle should be assigned to a button on the controller

    public double pointcloudDepthInfo;


    class Looper extends BaseIOIOLooper {

        /** The on-board LED. */
        private DigitalOutput led_;


        /**
         * Called every time a connection with IOIO has been established.
         * Typically used to open pins.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         *
         * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#setup()
         */
        @Override
        protected void setup() throws ConnectionLostException {
            toast("In IOIO Setup");
            led_ = ioio_.openDigitalOutput(0, true);

            pwm_speed_output = ioio_.openPwmOutput(3, 50); //motor channel 4: front left
            pwm_steering_output = ioio_.openPwmOutput(4, 50); //motor channel 3: back left

            pwm_speed_output.setPulseWidth(1500);
            pwm_steering_output.setPulseWidth(1500);


            showVersions(ioio_, "IOIO connected!");
        }

        /**
         * Called repetitively while the IOIO is connected.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         *
         * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#loop()
         */
        @Override
        public void loop() throws ConnectionLostException, InterruptedException {

            //if(pointcloud sees a wall)

            mRobot.update();

            if(pwm_speed > MAX_PWM) pwm_speed = MAX_PWM;
            else if(pwm_speed < MIN_PWM) pwm_speed = MIN_PWM;

            if(pwm_steering > MAX_PWM) pwm_steering = MAX_PWM;
            else if(pwm_steering < MIN_PWM) pwm_steering = MIN_PWM;

            ioio_.beginBatch();
            try
            {
                pwm_speed_output.setPulseWidth(pwm_speed);
                pwm_steering_output.setPulseWidth(pwm_steering);
                Thread.sleep(10);
            }
            catch (InterruptedException e){ ioio_.disconnect();}
            finally{ ioio_.endBatch();}


        }

        @Override
        public void disconnected() {
            toast("IOIO disconnected");
        }

        /**
         * Called when the IOIO is connected, but has an incompatible firmware version.
         *
         * @see ioio.lib.util.IOIOLooper#incompatible(IOIO)
         */
        @Override
        public void incompatible() {
            showVersions(ioio_, "Incompatible firmware version!");
        }

        private void showVersions(IOIO ioio, String title) {
            toast(String.format("%s\n" +
                            "IOIOLib: %s\n" +
                            "Application firmware: %s\n" +
                            "Bootloader firmware: %s\n" +
                            "Hardware: %s",
                    title,
                    ioio.getImplVersion(IOIO.VersionType.IOIOLIB_VER),
                    ioio.getImplVersion(IOIO.VersionType.APP_FIRMWARE_VER),
                    ioio.getImplVersion(IOIO.VersionType.BOOTLOADER_VER),
                    ioio.getImplVersion(IOIO.VersionType.HARDWARE_VER)));
        }


    }

    @Override
    protected IOIOLooper createIOIOLooper() {
        return new Looper();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_point_cloud);

        //IOIO
        pwm_speed = DEFAULT_PWM;
        pwm_steering = DEFAULT_PWM;

        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        CONTOUR_COLOR = new Scalar(255, 0, 0, 255);
        //To set color, find HSV values of desired color and convert each value to 1-255 scale
        //mDetector.setHsvColor(new Scalar(7, 196, 144)); // red
        //mDetector.setHsvColor(new Scalar(253.796875,222.6875,195.21875));
        mDetector.setHsvColor(new Scalar(7.015625,255.0,239.3125)); //bucket orange

        mRobot = new Robot(this);

        mAverageZTextView = (TextView) findViewById(R.id.average_z_textview);


        mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.gl_surface_view);

        textToSpeech = new TextToSpeech(MainActivity.this, new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int i) {
                textToSpeech.setLanguage(Locale.US);
            }
        });

        mPointCloudManager = new TangoPointCloudManager();
        mTangoUx = setupTangoUxAndLayout();
        mRenderer = new Scene(this);
        setupRenderer();

        //Set as topdown
        mRenderer.setTopDownView();
        mRenderer.renderVirtualObjects(true);

        tangoCameraPreview = (TangoTextureCameraPreview) findViewById(R.id.cameraPreview);

        mapInfo = new MapInfo();

        primaryColor = Color.parseColor("#FF3F51B5");
        primaryDark = Color.parseColor("#FF303F9F");

        mapInfo.setGrid(new int[GRID_SIZE][GRID_SIZE]);
        mapInfo.setCurrentCell(1,3,4);

        Window window = this.getWindow();
        window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);

        window.clearFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS);
        //Will Error setStatusBarColor due to MIN API lvl at 19
        window.setStatusBarColor(primaryDark);

        final Toolbar mToolBar = (Toolbar)findViewById(R.id.mainToolBar);
        //setSupportActionBar(mToolBar);
        //getSupportActionBar().setDisplayShowTitleEnabled(false);
        mToolBar.setTitleTextColor(Color.WHITE);
        mToolBar.setBackgroundColor(primaryColor);
        mToolBar.setTitle("");

        Button startPointButton = (Button) findViewById(R.id.setStartPoint);
        startPointButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Do something in response to button click
                mRenderer.setStartPoint(getCurrentPose());
                textToSpeech.speak("Start Point Set",TextToSpeech.QUEUE_FLUSH,null);
                Log.d("StartPoint","Startpoint Set at: " + getCurrentPose());
            }
        });

        Button endPointButton = (Button) findViewById(R.id.setEndPoint);
        endPointButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Do something in response to button click
                mRenderer.setEndPoint(getCurrentPose());
                Log.d("EndPoint","Endpoint Set at: " + getCurrentPose());
            }
        });



        ToggleButton toggle = (ToggleButton) findViewById(R.id.togglePointCloud);
        toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mRenderer.setThirdPersonView();
                    mRenderer.drawLineBtwnBuckets();
                } else {
                    mRenderer.setTopDownView();
                    mRenderer.removeLineBtwnBuckets();
                }
            }
        });

        ToggleButton toggleMotors = (ToggleButton) findViewById(R.id.toggleMotors);
        toggleMotors.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    toast("Speed"+": "+get_speed()+"Steer"+": "+get_steering());
                    set_speed(1500+800);
                    set_steering(1500);
                } else {
                    toast("Speed"+": "+get_speed()+"Steer"+": "+get_steering());
                    set_speed(1500);
                    set_steering(1500);
                }
            }
        });

        mMotorbar = (SeekBar)findViewById(R.id.motorBar); // make seekbar object
        mMotorbar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
                toast("MotorVal: "+motorSliderVal);
                set_speed(1500);
                set_steering(1500+motorSliderVal);

            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub

            }

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,
                                          boolean fromUser) {
                // TODO Auto-generated method stub
                motorSliderVal = progress;

            }
        });



        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {
                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setDisplayRotation();
                        mMapView.setFloorPlanData(mRenderer.getFloorPlanData());
                        setAndroidOrientation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {
                }
            }, null);
            if (hasPermission()) {
                if (null == savedInstanceState) {

                    //Instansiates the TensorflowView
                    //setFragment();
                }
            } else {
                requestPermission();
            }

        }
    }




    @Override
    protected void onStart() {
        super.onStart();
        mTangoUx.start();

        //new LongOperation().execute("");
        // Check and request camera permission at run time.
        bindTangoService();

    }

    @Override
    protected void onStop() {
        super.onStop();
        // Synchronize against disconnecting while the service is being used in the OpenGL
        // thread or in the UI thread.
        // NOTE: DO NOT lock against this same object in the Tango callback thread.
        // Tango.disconnect will block here until all Tango callback calls are finished.
        // If you lock against this object in a Tango callback thread it will cause a deadlock.
        synchronized (this) {
            try {
                mTangoUx.stop();
                mTango.disconnect();
                textToSpeech.stop();
                textToSpeech.shutdown();
                mIsConnected = false;
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        //new LongOperation().execute("");
    }

    /**
     * Initialize Tango Service as a normal Android Service.
     */
    private void bindTangoService() {
        // Initialize Tango Service as a normal Android Service. Since we call mTango.disconnect()
        // in onPause, this will unbind Tango Service, so every time onResume gets called we
        // should create a new Tango object.
        mTango = new Tango(MainActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                // Synchronize against disconnecting while the service is being used in the OpenGL
                // thread or in the UI thread.
                synchronized (MainActivity.this) {
                    try {
                        TangoSupport.initialize();
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                        mIsConnected = true;
                        setDisplayRotation();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                }
            }
        });
    }

    /**
     * Sets up the Tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango) {
        // Use the default configuration plus add depth sensing.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DRIFT_CORRECTION, true);
        tangoCameraPreview.connectToTangoCamera(mTango, TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
        //for now
        tangoCameraPreview.setRotation(90);
        return config;
    }

    /**
     * Set up the callback listeners for the Tango Service and obtain other parameters required
     * after Tango connection.
     * Listen to updates from the Point Cloud and Tango Events and Pose.
     */
    private void startupTango() {
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();

        framePairs.add(new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }

                mapPos = pose;


                /*
                TANGO POSE UPDATE FOR MAP HERE

                 */

                //mapInfo.setCurrentCell(pose);
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                if (mTangoUx != null) {
                    mTangoUx.updatePointCloud(pointCloud);
                }
                mPointCloudManager.updatePointCloud(pointCloud);

                final double currentTimeStamp = pointCloud.timestamp;
                final double pointCloudFrameDelta =
                        (currentTimeStamp - mPointCloudPreviousTimeStamp) * SECS_TO_MILLISECS;
                mPointCloudPreviousTimeStamp = currentTimeStamp;
                final double averageDepth = getAveragedDepth(pointCloud.points,
                        pointCloud.numPoints);

                mPointCloudTimeToNextUpdate -= pointCloudFrameDelta;

                if (mPointCloudTimeToNextUpdate < 0.0) {
                    mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
                    final String pointCountString = Integer.toString(pointCloud.numPoints);

                    calculateAveragedDepth(pointCloud.points, pointCloud.numPoints);

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            //mPointCountTextView.setText(pointCountString);
                            mAverageZTextView.setText(FORMAT_THREE_DECIMAL.format(averageDepth));
                        }
                    });
                }
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // We are not using onFrameAvailable for this application.
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                    tangoCameraPreview.onFrameAvailable();
                    bm[0] = tangoCameraPreview.getBitmap();
                    frameCount++;
                    Log.d("FPSTango",": "+frameCount);
                    if(frameCount == 15) {
                        frameCount=0;
                        scan(tangoCameraPreview.getBitmap());

                    }
                }
            }



            @Override
            public void onTangoEvent(TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.updateTangoEvent(event);
                }
            }
        });
    }

    public TangoPoseData getCurrentPose() {
        TangoPoseData currentTangoPos = TangoSupport.getPoseAtTime(0,
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE,
                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                mDisplayRotation);
        return currentTangoPos;
    }

    /**
     * Sets Rajawali surface view and its renderer. This is ideally called only once in onCreate.
     */
    public void setupRenderer() {
        mSurfaceView.setEGLContextClientVersion(2);
        mRenderer.getCurrentScene().registerFrameCallback(new ASceneFrameCallback() {
            @Override
            public void onPreFrame(long sceneTime, double deltaTime) {
                // NOTE: This will be executed on each cycle before rendering; called from the
                // OpenGL rendering thread.

                // Prevent concurrent access from a service disconnect through the onPause event.
                synchronized (MainActivity.this) {
                    // Don't execute any Tango API actions if we're not connected to the service.
                    if (!mIsConnected) {
                        return;
                    }

                    // Update point cloud data.
                    TangoPointCloudData pointCloud = mPointCloudManager.getLatestPointCloud();
                    if (pointCloud != null) {
                        // Calculate the depth camera pose at the last point cloud update.
                        TangoSupport.TangoMatrixTransformData transform =
                                TangoSupport.getMatrixTransformAtTime(pointCloud.timestamp,
                                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                        TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                        TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                        TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                        TangoSupport.ROTATION_IGNORED);
                        if (transform.statusCode == TangoPoseData.POSE_VALID) {
                            //Normally Update Points Goes Here But NOPE..
                            mRenderer.updatePointCloud(pointCloud,transform.matrix);
                        }
                    }

                    // Update current camera pose.
                    try {
                        // Calculate the device pose. This transform is used to display
                        // frustum in third and top down view, and used to render camera pose in
                        // first person view.
                        TangoPoseData lastFramePose = TangoSupport.getPoseAtTime(0,
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_DEVICE,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                mDisplayRotation);
                        if (lastFramePose.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updateCameraPose(lastFramePose);
                        }
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Could not get valid transform");
                    }
                }
            }

            @Override
            public boolean callPreFrame() {
                return true;
            }

            @Override
            public void onPreDraw(long sceneTime, double deltaTime) {

            }

            @Override
            public void onPostFrame(long sceneTime, double deltaTime) {

            }
        });
        mSurfaceView.setSurfaceRenderer(mRenderer);
    }

    /**
     * Sets up TangoUX and sets its listener.
     */
    private TangoUx setupTangoUxAndLayout() {
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);
        return tangoUx;
    }

    /*
    * Set a UxExceptionEventListener to be notified of any UX exceptions.
    * In this example we are just logging all the exceptions to logcat, but in a real app,
    * developers should use these exceptions to contextually notify the user and help direct the
    * user in using the device in a way Tango Service expects it.
    * <p>
    * A UxExceptionEvent can have two statuses: DETECTED and RESOLVED.
    * An event is considered DETECTED when the exception conditions are observed, and RESOLVED when
    * the root causes have been addressed.
    * Both statuses will trigger a separate event.
    */
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {
        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            String status = uxExceptionEvent.getStatus() == UxExceptionEvent.STATUS_DETECTED ?
                    UX_EXCEPTION_EVENT_DETECTED : UX_EXCEPTION_EVENT_RESOLVED;

            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE) {
                Log.i(TAG, status + "Device lying on surface");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                Log.i(TAG, status + "Too few depth points");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                Log.i(TAG, status + "Too few features");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                Log.i(TAG, status + "Invalid poses in MotionTracking");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                Log.i(TAG, status + "Moving too fast");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_OVER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Over Exposed");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_UNDER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Under Exposed");
            }
        }
    };

    /**
     * First Person button onClick callback.
     */
    public void onFirstPersonClicked(View v) {
        mRenderer.setFirstPersonView();
    }

    /**
     * Third Person button onClick callback.
     */
    public void onThirdPersonClicked(View v) {
        mRenderer.setThirdPersonView();
    }

    /**
     * Top-down button onClick callback.
     */
    public void onTopDownClicked(View v) {
        mRenderer.setTopDownView();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mRenderer.onTouchEvent(event);
        return true;
    }

    /**
     * Calculates the average depth from a point cloud buffer.
     *
     * @param pointCloudBuffer
     * @param numPoints
     * @return Average depth.
     */
    private float getAveragedDepth(FloatBuffer pointCloudBuffer, int numPoints) {
        float totalZ = 0;
        float averageZ = 0;
        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {
                totalZ = totalZ + pointCloudBuffer.get(i);
            }
            averageZ = totalZ / numPoints;
        }
        return averageZ;
    }

    /**
     * Query the display's rotation.
     */
    private void setDisplayRotation() {
        Display display = getWindowManager().getDefaultDisplay();
        mDisplayRotation = display.getRotation();
    }

    /**
     * Display toast on UI thread.
     *
     * @param resId The resource id of the string resource to use. Can be formatted text.
     */
    private void showsToastAndFinishOnUiThread(final int resId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(MainActivity.this,
                        getString(resId), Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }

    public void introDialog(){
        new MaterialDialog.Builder(MainActivity.this)
                .theme(Theme.DARK)
                //.iconRes(R.mipmap.ic_launcher)
                .positiveColor(Color.WHITE)
                .title(R.string.app_name)
                .titleGravity(GravityEnum.CENTER)
                .titleColor(Color.WHITE)
                .content("Created by Rafael Szuminski")
                .positiveText("Continue")
                .backgroundColor(Color.rgb(48,48,48))
                .btnSelector(R.drawable.md_btn_selector_custom, DialogAction.POSITIVE)
                .contentColor(Color.WHITE)
                .show();
    }




    @Override
    public boolean onGenericMotionEvent(MotionEvent event) {
        // Check that the event came from a game controller
        try {
            if ((event.getSource() & InputDevice.SOURCE_JOYSTICK) ==
                    InputDevice.SOURCE_JOYSTICK &&
                    event.getAction() == MotionEvent.ACTION_MOVE) {

                // Process all historical movement samples in the batch
                final int historySize = event.getHistorySize();

                // Process the movements starting from the
                // earliest historical position in the batch
                for (int i = 0; i < historySize; i++) {
                    // Process the event at historical position i
                    processJoystickInput(event, i);
                }

                // Process the current movement sample in the batch (position -1)
                processJoystickInput(event, -1);
                return true;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return super.onGenericMotionEvent(event);
    }

    /*
     * I'm not gonna mess with this, I'll take your word for it that it works
     * However, I don't think we need it because of how the processJoystickInput method works
    */
    private static float getCenteredAxis(MotionEvent event,
                                         InputDevice device, int axis, int historyPos) {
        final InputDevice.MotionRange range =
                device.getMotionRange(axis, event.getSource());

        // A joystick at rest does not always report an absolute position of
        // (0,0). Use the getFlat() method to determine the range of values
        // bounding the joystick axis center.
        if (range != null) {
            final float flat = range.getFlat();
            final float value =
                    historyPos < 0 ? event.getAxisValue(axis) :
                            event.getHistoricalAxisValue(axis, historyPos);

            // Ignore axis values that are within the 'flat' region of the
            // joystick axis center.
            if (Math.abs(value) > flat) {
                return value;
            }
        }
        return 0;
    }

    //Making the joystick work, this allows X and Y Axis controls on the stick to be detected.
    //This also enables us to then add keyevents using Dennys math to check the ammount of force each wheel must
    //turn.
    //   [LStick]:Forward and backwards movement.    [Rstick]: left and right turning.
    //
    private void processJoystickInput(MotionEvent event,
                                      int historyPos) {
        InputDevice mInputDevice = event.getDevice();

        float x = getCenteredAxis(event, mInputDevice,
                MotionEvent.AXIS_Z, historyPos);
        float y = getCenteredAxis(event, mInputDevice,
                MotionEvent.AXIS_Y, historyPos);
        Log.d("Controller", "AnalogX:" + x);
        Log.d("Controller", "AnalogY:" + y);

        // Channel 1 STOP:64 Channel 2 STOP:192

        // x = x+y
        int rightMotor = 0, leftMotor = 0;

        if (Math.round(Math.abs(55 * y)) > 0) {
            rightMotor += Math.round(55 * y);
            leftMotor += Math.round(55 * y);
            if (x > 0) {
                rightMotor -= Math.round(55 * (-x));
                int tempSpd = leftMotor+rightMotor;
                set_speed(1500+tempSpd);
            } else if (x < 0) {
                leftMotor += Math.round(55 * (-x));
                int tempSpd = leftMotor+rightMotor;
                set_speed(1500+800-tempSpd);
            }
        } else {
            rightMotor -= Math.round(55 * (-x));
            leftMotor += Math.round(55 * (-x));
        }

        Log.d("Controller", "Motor Left:" + leftMotor);
        Log.d("Controller", "Motor Right:" + rightMotor);


    }




    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i("activity cycle","main activity being destroyed");

    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {

        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case PERMISSIONS_REQUEST: {
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED
                        && grantResults[1] == PackageManager.PERMISSION_GRANTED) {

                } else {
                    requestPermission();
                }
            }
        }
    }

    private boolean hasPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            return checkSelfPermission(PERMISSION_CAMERA) == PackageManager.PERMISSION_GRANTED && checkSelfPermission(PERMISSION_STORAGE) == PackageManager.PERMISSION_GRANTED;
        } else {
            return true;
        }
    }

    private void requestPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (shouldShowRequestPermissionRationale(PERMISSION_CAMERA) || shouldShowRequestPermissionRationale(PERMISSION_STORAGE)) {
                Toast.makeText(MainActivity.this, "Camera AND storage permission are required for this demo", Toast.LENGTH_LONG).show();
            }
            requestPermissions(new String[] {PERMISSION_CAMERA, PERMISSION_STORAGE}, PERMISSIONS_REQUEST);
        }
    }


    private void setAndroidOrientation() {
        Display display = getWindowManager().getDefaultDisplay();
        Camera.CameraInfo depthCameraInfo = new Camera.CameraInfo();
        Camera.getCameraInfo(1, depthCameraInfo);

        int depthCameraRotation = Surface.ROTATION_0;
        switch(depthCameraInfo.orientation) {
            case 90:
                depthCameraRotation = Surface.ROTATION_90;
                break;
            case 180:
                depthCameraRotation = Surface.ROTATION_180;
                break;
            case 270:
                depthCameraRotation = Surface.ROTATION_270;
                break;
        }

        mDepthCameraToDisplayRotation = display.getRotation() - depthCameraRotation;
        if (mDepthCameraToDisplayRotation < 0) {
            mDepthCameraToDisplayRotation += 4;
        }
    }

    private void toast(final String message) {
        final Context context = this;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(context, message, Toast.LENGTH_LONG).show();
            }
        });
    }

    //Scan for QR code and save information to phone
    public String scan(Bitmap bMap) {
        int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];
        //copy pixel data from the Bitmap into the 'intArray' array
        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(),intArray);

        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
        Reader reader = new QRCodeReader();

        String text;

        try {
            Result result = reader.decode(bitmap);
            text = result.getText();

            //Toasts the Info
            toast(text+" "+result);
            textToSpeech.speak(text,TextToSpeech.QUEUE_FLUSH,null);
            mRenderer.setTrueMarker();
            Calendar calendar = Calendar.getInstance();
            java.util.Date now = calendar.getTime();
            java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
            String time = currentTimestamp.toString();
            String info = text;

            try {
                File newFolder = new File(Environment.getExternalStorageDirectory(), "RescueRobotics");
                if (!newFolder.exists()) {
                    newFolder.mkdir();
                }
                try {
                    File file = new File(newFolder, time + ".txt");
                    file.createNewFile();
                    FileOutputStream fos=new FileOutputStream(file);
                    try {
                        byte[] b = info.getBytes();
                        fos.write(b);
                        fos.close();
                        ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
                        toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
                    } catch (IOException e) {
                        Log.e("app.main","Couldn't write to SD");
                    }
                } catch (Exception ex) {
                    Log.e("app.main","Couldn't write to SD");
                }
            } catch (Exception e) {
                Log.e("app.main","Couldn't write to SD");
            }
            Log.i("rescue robotics",text);
            return text;
        } catch (NotFoundException e) {
            e.printStackTrace();
            text = "no code found";
        } catch (ChecksumException e) {
            e.printStackTrace();
            text =  "checksum error";
        } catch (FormatException e) {
            e.printStackTrace();
            text = "format error";
        }
        Log.i("rescue robotics",text);

        return text;
    }

    //IOIO Funcs
    public synchronized void set_speed(int speed)
    {
        pwm_speed = speed;
        if(pwm_speed > MAX_PWM) pwm_speed = MAX_PWM;
        else if(pwm_speed < MIN_PWM) pwm_speed = MIN_PWM;
    }

    public synchronized void set_steering(int steering)
    {
        pwm_steering = steering;
        if(pwm_steering > MAX_PWM) pwm_steering = MAX_PWM;
        else if(pwm_steering < MIN_PWM) pwm_steering = MIN_PWM;
    }

    public synchronized int get_speed()
    {
        return pwm_speed;
    }

    public synchronized int get_steering()
    {
        return pwm_steering;
    }

    private float calculateAveragedDepth(FloatBuffer pointCloudBuffer, int numPoints) {
        float totalZ = 0;
        float averageZ = 0;
        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {
                totalZ = totalZ + pointCloudBuffer.get(i);
            }
            averageZ = totalZ / numPoints;
        }
        return averageZ;
    }

}
