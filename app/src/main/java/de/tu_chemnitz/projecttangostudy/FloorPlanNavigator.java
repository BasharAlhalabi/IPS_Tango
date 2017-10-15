package de.tu_chemnitz.projecttangostudy;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.hardware.display.DisplayManager;
import android.opengl.GLSurfaceView;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tango.reconstruction.TangoPolygon;
//import com.google.atap.tango.ux.TangoUx;
//import com.google.atap.tango.ux.TangoUxLayout;
//import com.google.atap.tango.ux.UxExceptionEvent;
//import com.google.atap.tango.ux.UxExceptionEventListener;
import com.google.atap.tango.ux.TangoUx;
import com.google.atap.tango.ux.TangoUxLayout;
import com.google.atap.tango.ux.UxExceptionEvent;
import com.google.atap.tango.ux.UxExceptionEventListener;
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
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.tangosupport.TangoSupport;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

import static com.google.atap.tango.ux.TangoUx.TYPE_HOLD_POSTURE_NONE;
import static com.google.atap.tango.ux.TangoUx.TYPE_HOLD_POSTURE_UP;
import static com.google.atap.tango.ux.UxExceptionEvent.TYPE_LYING_ON_SURFACE;

/**
 * Created by Bashar Al Halabi on 12.04.2017.
 * Idea: from com.projecttango.examples.java.floorplanreconstruction
 * Note: Calculation style does not maintain restrictions of solid OOP principles intentionally at the time being for experimental purposes
 */

public class FloorPlanNavigator extends Activity implements _2D_RenderingView.DrawingCallback {


    private static final String TAG = FloorPlanNavigator.class.getSimpleName();

    //Global variables-----------------------
    public static boolean height_manually_adjusted = false;
    private static final String CAMERA_PERMISSION = Manifest.permission.CAMERA;
    private static final int CAMERA_PERMISSION_CODE = 0;
    static SeekBar map_scale_seekBar;
    static int _loc_ctr = 0;
    Dialog advanced_dialog;
    static boolean allow_update_cam_poses = false; //for init. purposes

    //-----------------------------

    public static boolean different_floors_heights = false; //every height has different height. Not studied deeply!
    final static int MAX_FLOOR_NUM = 5;
    static double[] estimated_floor_heightS; //can hold MAX_FLOOR_NUM floors
    static boolean service_has_started = false;
    static ArrayList<Pose_Structure> pose_keeper = new ArrayList<>();
    static ArrayList<FloorSwitch_Pose_Structure> floorSwitch_pose_keeper = new ArrayList<>();

    static boolean height_auto_adjustment_mode = false;

    static Boolean map_calibration_phase = true;
    static Boolean user_allows_Map = true;
    static Boolean user_allows_Scale_test = false;
    static Boolean user_allows_DriftCorrection = true;
    static Boolean user_allows_TangoUx = true;
    static Boolean user_allows_openGLView = true;
    static Boolean user_allows_vibration = true;
    static Boolean dev_allows_vibration_yet = true;

    static double limit_far = 1.5;
    static double limit_near = 0.3;

    double current_device_height = 0.0;
    public static float init_scale_factor = 0.0f;
    public static double realTime_height = 0.0;
    public static double realTime_Hands_height = 0.0;
    static double floorHeight = 2.7; //2.5 meter
    static double handHeight = 0.2; // 1.45
    static double height_to_switch = 0.0;
    static int startFloorNum = 1;
    static int currFloorNum = 1;
    static double avg_depth = 0.0;
    static double walked_distance = 0.0;

    static Activity UIThread_accessibility_providing_activity;
    static TextView mAvg_depthText;
    static boolean show_avg_depth_log = false;
    static boolean show_grid = true;

    //shared preferences settings:
    static final String PREFS_NAME = "Tango_Pref";
    static final String PREFS_FLOOR_HEIGHT = "floorHeight";

    //if diff. floor heights
    static final String PREFS_FLOOR_HEIGHT1 = "estimated_floor_height1";
    static final String PREFS_FLOOR_HEIGHT2 = "estimated_floor_height2";
    static final String PREFS_FLOOR_HEIGHT3 = "estimated_floor_height3";
    static final String PREFS_FLOOR_HEIGHT4 = "estimated_floor_height4";
    static final String PREFS_FLOOR_HEIGHT5 = "estimated_floor_height5";

    static final String PREFS_FAR = "limit_far";
    static final String PREFS_GRID_VISIBILITY = "GRID_VISIBLE";
    static final String PREFS_UNI_HEIGHTS = "PREFS_UNI_HEIGHTS";
    static final String PREFS_VIBRATION = "vibration";
    static final String PREFS_PERSON_HAND_HEIGHT = "PREFS_PERSON_HAND_HEIGHT";
    static final String PREFS_START_FLOOR = "PREFS_START_FLOOR";
    static final String PREFS_INIT_SCALE_FACTOR = "PREFS_INIT_SCALE_FACTOR";
    static final String PREFS_OPENGL_VIEW = "PREFS_OPENGL_VIEW";
    static final String PREFS_TANGO_UX = "PREFS_TANGO_UX";
    static final String PREFS_AREA_LEARNING = "PREFS_AREA_LEARNING";

    //---------------------------------
    //--------------VID OpenGL-------------------

    private GLSurfaceView mOpenGLSurfaceView;
    private Video_Renderer mOpenGL_Renderer;
    private int mConnectedTextureIdGlThread = INVALID_TEXTURE_ID;
    private static final int INVALID_TEXTURE_ID = 0;
    private AtomicBoolean mIsFrameAvailableTangoThread = new AtomicBoolean(false);
    private static final String sTimestampFormat = "Timestamp: %f";
    private FrameLayout openGLSurfaceFrameLayout;

    //----------------------------------
    //----------------AL------------------

    public static final int REQUEST_CODE_TANGO_PERMISSION = 3457;
    public static boolean isDriftCorrection = true;
    public static boolean isLearningMode = false;
    public static boolean isLoadAdf = false; // Drift-correction mode is incompatible with learning mode or ADF loading.

    boolean mIsRelocalized = false;
    private static final double UPDATE_INTERVAL_MS = 100.0;
    private double mPreviousPoseTimeStamp;
    private double mTimeToNextUpdate = UPDATE_INTERVAL_MS;
    private static final int SECS_TO_MILLISECS = 1000;
    private final Object mSharedLock = new Object();
    // Long-running task to save the ADF.
    private SaveAdfTask mSaveAdfTask;

    //-----------------------------------
    //----------TangoUX------------------
    private TangoUx mTangoUx;

    //------------------------------------
    private FloorManager mFloorManager; //to cont. processing Tango poses
    protected Tango mTango;
    private TangoConfig mConfig;
    private boolean mIsConnected = false;
    private boolean mIsPaused;
    public static _2D_RenderingView m2D_RenderingView;
    private TextView mDistText, mFloor_number, mTVHeight, mTVLocalization, textView_rt_height_val, textView_rt_hand_height_val, textView_rest_h_2_switch_val, textView_test_scale_val;
    private CheckBox cb_map_mode, cb_sclae_test_mode;
    private int mDisplayRotation = 0;

    //OpenCv:
    public static boolean OpenCv_loaded = false;

    //End global vars.------------------------------

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        estimated_floor_heightS = new double[MAX_FLOOR_NUM];
        handle_shared_prefs();

        setContentView(R.layout.activity_floor_plan_navigator);
        openGLSurfaceFrameLayout = (FrameLayout) findViewById(R.id.main_layout);

        UIThread_accessibility_providing_activity = this;
        make_start_dialog(UIThread_accessibility_providing_activity);

        //ADF permissions:
        startActivityForResult(  Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_ADF_LOAD_SAVE), Tango.TANGO_INTENT_ACTIVITYCODE);

        map_scale_seekBar = (SeekBar) findViewById(R.id.seekBar_map_scale);
        map_scale_seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {

                if(i != 0)
                    if(!user_allows_Scale_test)
                        _2D_RenderingView.last_scale = (float)i/100.0f;
                    else _2D_RenderingView.last_test_scale = (float)i/100.0f;
                else
                {
                    reset_seekBar();
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                //do nothing
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

                if(FloorManager.camera_corner_pos_initialized && user_allows_Scale_test)
                {
                    //Store prefs:
                    SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                    SharedPreferences.Editor editor = settings.edit();
                    editor.putFloat(PREFS_INIT_SCALE_FACTOR, _2D_RenderingView.last_test_scale);
                    editor.apply();

                }
            }
        });

    }

    public static void reset_seekBar() //reset scaling
    {
        map_scale_seekBar.setProgress(250); //reset global scale
        _2D_RenderingView.last_scale = (float) 250/100.0f;
    }

    private void handle_shared_prefs() {

        //Store prefs:
        SharedPreferences settings = getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
        if (!settings.contains(PREFS_FLOOR_HEIGHT)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FLOOR_HEIGHT, 2.5f);
            editor.apply();
        } else {
            floorHeight = settings.getFloat(PREFS_FLOOR_HEIGHT, 0);
        }

        if (!settings.contains(PREFS_FAR)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FAR, 1.5f);
            editor.apply();
        } else {
            limit_far = settings.getFloat(PREFS_FAR, 0);
        }

        if (!settings.contains(PREFS_GRID_VISIBILITY)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putBoolean(PREFS_GRID_VISIBILITY, true);
            editor.apply();
        } else {
            show_grid = settings.getBoolean(PREFS_GRID_VISIBILITY, true);
        }


        if (!settings.contains(PREFS_UNI_HEIGHTS)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putBoolean(PREFS_UNI_HEIGHTS, true);
            editor.apply();
        } else {
            different_floors_heights = settings.getBoolean(PREFS_UNI_HEIGHTS, true);
        }

        if (!settings.contains(PREFS_VIBRATION)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putBoolean(PREFS_VIBRATION, true);
            editor.apply();
        } else {
            user_allows_vibration = settings.getBoolean(PREFS_VIBRATION, true);
        }

        if (!settings.contains(PREFS_PERSON_HAND_HEIGHT)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_PERSON_HAND_HEIGHT, 1.45f);
            editor.apply();
        } else {
            handHeight = settings.getFloat(PREFS_PERSON_HAND_HEIGHT, 0);
        }

        if (!settings.contains(PREFS_START_FLOOR)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putInt(PREFS_START_FLOOR, 2);
            editor.apply();
        } else {
            startFloorNum = settings.getInt(PREFS_START_FLOOR, 0);

        }

        if (!settings.contains(PREFS_OPENGL_VIEW)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putBoolean(PREFS_OPENGL_VIEW, true);
            editor.apply();
        } else {
            user_allows_openGLView = settings.getBoolean(PREFS_OPENGL_VIEW, true);
        }

        if (!settings.contains(PREFS_TANGO_UX)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putBoolean(PREFS_TANGO_UX, true);
            editor.apply();
        } else {
            user_allows_TangoUx = settings.getBoolean(PREFS_TANGO_UX, true);
        }

        if (!settings.contains(PREFS_AREA_LEARNING)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putBoolean(PREFS_AREA_LEARNING, true);
            editor.apply();
        } else {
            user_allows_DriftCorrection = settings.getBoolean(PREFS_AREA_LEARNING, true);
            if(user_allows_DriftCorrection)
            {
                isDriftCorrection = true; //instantaneous corrections (run time, no ability to store or load ADFs)
                isLearningMode = false;
                isLoadAdf = false;
                allow_update_cam_poses = false;
            }
            else
            {
                isDriftCorrection = false;
                isLearningMode = true; //saving a new ADF is done only after localized if the current session based on a loaded ADF........after a fresh start (not based on ADF) can save ADF. By starting based on loaded ADF, a new ADF can only be saved after localized.
                isLoadAdf = true; //will load an ADF if found and try to calibrate it with environment (freezes all until calibration happens)
                allow_update_cam_poses = true;
            }
        }

        if (!settings.contains(PREFS_FLOOR_HEIGHT1)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FLOOR_HEIGHT1, 0);
            editor.apply();
        } else {
            estimated_floor_heightS[0] = settings.getFloat(PREFS_FLOOR_HEIGHT1, 0);
        }

        if (!settings.contains(PREFS_FLOOR_HEIGHT2)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FLOOR_HEIGHT2, 0);
            editor.apply();
        } else {
            estimated_floor_heightS[1] = settings.getFloat(PREFS_FLOOR_HEIGHT2, 0);
        }

        if (!settings.contains(PREFS_FLOOR_HEIGHT3)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FLOOR_HEIGHT3, 0);
            editor.apply();
        } else {
            estimated_floor_heightS[2] = settings.getFloat(PREFS_FLOOR_HEIGHT3, 0);
        }

        if (!settings.contains(PREFS_FLOOR_HEIGHT4)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FLOOR_HEIGHT4, 0);
            editor.apply();
        } else {
            estimated_floor_heightS[3] = settings.getFloat(PREFS_FLOOR_HEIGHT4, 0);
        }

        if (!settings.contains(PREFS_FLOOR_HEIGHT5)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_FLOOR_HEIGHT5, 0);
            editor.apply();
        } else {
            estimated_floor_heightS[4] = settings.getFloat(PREFS_FLOOR_HEIGHT5, 0);
        }

        if (!settings.contains(PREFS_INIT_SCALE_FACTOR)) {
            SharedPreferences.Editor editor = settings.edit();
            editor.putFloat(PREFS_INIT_SCALE_FACTOR, 0);
            editor.apply();
        } else {
            init_scale_factor = settings.getFloat(PREFS_INIT_SCALE_FACTOR, 0);
            _2D_RenderingView.last_test_scale = init_scale_factor;
        }

        currFloorNum = startFloorNum;

    }


    @Override
    protected void onStart() {
        super.onStart();

        do_onStart();

    }


    @Override
    protected void onPause() {
        super.onPause();

            if (advanced_dialog != null)
                advanced_dialog.dismiss();
            mIsPaused = true;

            // Clear the relocalization state: we don't know where the device will be since our app will be paused.
            mIsRelocalized = false;

        //Focusing on content more than app life cycle: if interrupted kill the app and start again.
    }

    private void do_onPause_Vid() {

        mOpenGLSurfaceView.onPause();
        try {
            mTango.disconnectCamera(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
            // We need to invalidate the connected texture ID so that we cause a
            // re-connection in the OpenGL thread after resume
            mConnectedTextureIdGlThread = INVALID_TEXTURE_ID;
            //mTango.disconnect();
            mIsConnected = false;
        } catch (TangoErrorException e) {
            Log.e(TAG, getString(R.string.exception_tango_error), e);
        }

        Log.i("OnPause", "Paused!");
    }


    @Override
    protected void onResume() {
        super.onResume();

            if (mIsPaused && !service_has_started) {
                make_start_dialog(this);
                mIsPaused = false;
            }

        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

            Log.i("OnResume", "Resumed!");

    }

    private void do_onResume_Vid() {

        //for Video preview
        if (mOpenGLSurfaceView != null) {
            mOpenGLSurfaceView.onResume();
            // Set render mode to RENDERMODE_CONTINUOUSLY to force getting onDraw callbacks until the
            // Tango service is properly set-up and we start getting onFrameAvailable callbacks.
            mOpenGLSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        }
    }

    /**
     * Initialize Tango Service as a normal Android Service.
     */
    private void bindTangoService() {
        // Initialize Tango Service as a normal Android Service.
        // Since we call mTango.disconnect() in onPause, this will unbind Tango Service,
        // so every time when onResume gets called, we should create a new Tango object.
        mTango = new Tango(FloorPlanNavigator.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready,
            // this Runnable will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only
            // when there is no UI thread changes involved.
            @Override
            public void run() {
                synchronized (FloorPlanNavigator.this) {
                    try {

                        TangoSupport.initialize();

                        mConfig = setupTangoConfig(mTango);

                        if(mTangoUx != null && user_allows_TangoUx) {
                            mTangoUx.start(new TangoUx.StartParams());
                            Log.i("mTangoUx", "Started!");
                        }
                        else mTangoUx = null;

                        mTango.connect(mConfig);
                        startupTango();
                        mIsConnected = true;
                        mIsPaused = false;

                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {

                                setupVideoRenderer();
                                setDisplayRotation();
                            }
                        });

                    } catch (TangoOutOfDateException e) {
                        if (mTangoUx != null && user_allows_TangoUx) {
                            mTangoUx.showTangoOutOfDate();
                        }
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                        showsToastAndFinishOnUiThread(R.string.exception_out_of_date);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                    catch (Exception ex){ex.printStackTrace();}
                }
            }
        });

    }

    /**
     * Set the display rotation. //onCreate we added listener!
     */
    private void setDisplayRotation() {
        Display display = getWindowManager().getDefaultDisplay();
        mDisplayRotation = display.getRotation();

        //Video:
        // We also need to update the camera texture UV coordinates. This must be run in the OpenGL thread.
        mOpenGLSurfaceView.queueEvent(new Runnable() {
            @Override
            public void run() {
                if (FloorPlanNavigator.service_has_started) {
                    mOpenGL_Renderer.updateColorCameraTextureUv(mDisplayRotation);
                }
            }
        });


    }

    /**
     * Sets up the tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango) {
        // Use default configuration for Tango Service, plus color camera, low latency
        // IMU integration, depth, smooth pose and dataset recording.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT); //prev: CONFIG_TYPE_DEFAULT

        // NOTE: Low latency integration is necessary to achieve a precise alignment of virtual
        // objects with the RBG image and produce a good AR effect.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);

        // Motion tracking activating
        //Not for Now: config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        // Tango service should automatically attempt to recover when it enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);

        // Create a new Tango Configuration and enable the Camera API
        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, true);

        //Area learning modes:
        // Check the learning mode & Set learning mode to config.
        if (isLoadAdf && isLearningMode && !isDriftCorrection) {

            // Check for ADFs.
            {
                ArrayList<String> fullUuidList;
                // Returns a list of ADFs with their UUIDs.
                fullUuidList = tango.listAreaDescriptions();
                // Load the latest ADF if ADFs are found.
                if (fullUuidList != null && fullUuidList.size() > 0) {
                    config.putString(TangoConfig.KEY_STRING_AREADESCRIPTION, fullUuidList.get(fullUuidList.size() - 1));
                    allow_update_cam_poses = true;
                    showsToastOnUiThread("ADF file(s) found", this);
                    Log.i("ADF ok", "ADFs OK");
                } else {
                    Log.i("No ADFs found!", "No ADFs found!");
                    showsToastOnUiThread("No ADF files are found.", this);
                }
            }

            config.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);

        } else if (isDriftCorrection) {
            allow_update_cam_poses = false;
            config.putBoolean(TangoConfig.KEY_BOOLEAN_DRIFT_CORRECTION, true);
        }

        return config;
    }

    @Override
    protected void onStop() {
        super.onStop();

        do_stop_Tango_service();
    }

    private void do_stop_Tango_service() {

        // Synchronize against disconnecting while the service is being used in the OpenGL thread or in the UI thread.
        synchronized (this) {
            try {
                if (mIsConnected) {
                    service_has_started = false;
                    mFloorManager.stopFloorplanning();
                    do_onPause_Vid();
                    mTango.disconnect();

                    if(mTangoUx != null) {
                        mTangoUx.stop();
                    }
                    mFloorManager.resetFloorplan();
                    mFloorManager.release();
                    mIsConnected = false;
                    mIsPaused = true;
                }
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }

        Log.i("do_stop_Tango_service", "do_stop_Tango_service!");
    }


    @Override
    public void onPreDrawing() {
        if (service_has_started) {
            try {
                // Synchronize against disconnecting while using the service.
                synchronized (FloorPlanNavigator.this) {
                    // Don't execute any tango API actions if we're not connected to
                    // the service
                    if (!mIsConnected) {
                        Log.w(TAG, "Tango service reconnecting!");

                           // showsToastOnUiThread(R.string.exception_tango_service_disconnected, UIThread_accessibility_providing_activity);
                        return;
                    }

                    if(!isDriftCorrection && !isLearningMode) {
                        //Here we get valid poses with the speed of drawing thread and not the speed of Tango pose available callbacks:
                        //Anyway, the speed of drawing thread is 100Hz, which is max freq. for Tango pose available
                        // Calculate the device pose in OpenGL engine (Y+ up). // Based on starting time: 0.0 (starting height)
                        //This allows to align the pose timestamps with the video frames:
                        TangoPoseData devicePose = TangoSupport.getPoseAtTime(0.0,
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_DEVICE,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL, mDisplayRotation);


                        if (mTangoUx != null && user_allows_TangoUx) {
                            mTangoUx.updatePoseStatus(devicePose.statusCode);
                        } else if (mTangoUx != null) {
                            mTangoUx.stop();
                        }

                        //Passing the pose:
                        if (devicePose.statusCode == TangoPoseData.POSE_VALID) {
                            // Extract position and rotation around Z.
                            float[] devicePosition = devicePose.getTranslationAsFloats();
                            float[] deviceOrientation = devicePose.getRotationAsFloats();
                            float yawRadians = yRotationFromQuaternion(deviceOrientation[0],
                                    deviceOrientation[1], deviceOrientation[2],
                                    deviceOrientation[3]);
                            float pitchRadians = xRotationFromQuaternion(deviceOrientation[0],
                                    deviceOrientation[1], deviceOrientation[2],
                                    deviceOrientation[3]);

                            //passing camera data calculated from pose date up here (No Y: Z frontwards)
                            m2D_RenderingView.updateCameraMatrix(devicePosition[0], -devicePosition[2], yawRadians, pitchRadians);
                            //passing full valid pose data of a frame from here (Height of Cam: Y)
                            mFloorManager.setAvailablePose(devicePose);
                            //loging pose data:
                            logPose(devicePose);
                        } else {
                            Log.w(TAG, "Can't get last normal device pose");
                        }
                    }
                    else { //Area Learning/ drift corr.

                        //Activate area learning to correct movements drifting
                        TangoPoseData devicePose_DC = TangoSupport.getPoseAtTime(0.0,
                                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                                TangoPoseData.COORDINATE_FRAME_DEVICE,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL, mDisplayRotation);

                        if (mTangoUx != null && user_allows_TangoUx) {
                            mTangoUx.updatePoseStatus(devicePose_DC.statusCode);
                        } else if (mTangoUx != null) {
                            mTangoUx.stop();
                        }

                        // Make sure to have atomic access to Tango Data so that UI loop doesn't interfere
                        // while Pose call back is updating the data.
                        synchronized (mSharedLock) {

                            // Check for Device wrt ADF pose, Device wrt Start of Service pose, Start of
                            // Service wrt ADF pose (This pose determines if the device is relocalized or not).
                            if (devicePose_DC.statusCode == TangoPoseData.POSE_VALID) {
                                if (devicePose_DC.baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION && devicePose_DC.targetFrame == TangoPoseData.COORDINATE_FRAME_DEVICE) {
                                    mIsRelocalized = true;

                                    if (mIsRelocalized) {

                                        //corrected poses count, secondary important.
                                        _loc_ctr++;

                                        float[] devicePosition = devicePose_DC.getTranslationAsFloats();
                                        float[] deviceOrientation = devicePose_DC.getRotationAsFloats();
                                        float yawRadians = yRotationFromQuaternion(deviceOrientation[0],
                                                deviceOrientation[1], deviceOrientation[2],
                                                deviceOrientation[3]);
                                        float pitchRadians = xRotationFromQuaternion(deviceOrientation[0],
                                                deviceOrientation[1], deviceOrientation[2],
                                                deviceOrientation[3]);
                                        m2D_RenderingView.updateCameraMatrix(devicePosition[0], -devicePosition[2], yawRadians, pitchRadians);
                                        //passing full valid pose data of a frame from here (Height of Cam: Y)
                                        mFloorManager.setAvailablePose(devicePose_DC);
                                        //loging pose data:
                                        logPose(devicePose_DC);
                                    }

                                } else {
                                    mIsRelocalized = false;
                                }
                            } else {
                                Log.w(TAG, "Can't get last corrected device pose");
                            }
                        }

                        final double deltaTime = (devicePose_DC.timestamp - mPreviousPoseTimeStamp) * SECS_TO_MILLISECS;
                        mPreviousPoseTimeStamp = devicePose_DC.timestamp;
                        mTimeToNextUpdate -= deltaTime;

                        if (mTimeToNextUpdate < 0.0) {
                            mTimeToNextUpdate = UPDATE_INTERVAL_MS;

                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    synchronized (mSharedLock) {
                                        mTVLocalization.setText(String.valueOf(mIsRelocalized) + ": " + String.valueOf(_loc_ctr));
                                    }
                                }
                            });

                        }
                    }

                }

                updateGUI();

            } catch (TangoErrorException e) {
                Log.e(TAG, "Tango error while querying device pose.", e);
            } catch (TangoInvalidException e) {
                Log.e(TAG, "Tango exception while querying device pose.", e);
            }
        }

    }


    /**
     * Calculates the rotation around X (Pitch) from the given quaternion.
     */
    private static float xRotationFromQuaternion(float x, float y, float z, float w) {
        return (float) Math.atan2(2 * (w * x - y * z), w * (w + y) - x * (z + x));
    }

    /**
     * Calculates the rotation around Y (yaw) from the given quaternion.
     */
    private static float yRotationFromQuaternion(float x, float y, float z, float w) {
        return (float) Math.atan2(2 * (w * y - x * z), w * (w + x) - y * (z + y));
    }

    /**
     * Calculates the rotation around Z (Roll) from the given quaternion. (?? not tested yet)
     */
    private static float zRotationFromQuaternion(float x, float y, float z, float w) {
        return (float) Math.atan2(2 * (w * z - y * x), w * (w + x) - z * (y + x));
    }


    /**
     * Calculate the total walked distance and update the text field with that information. (b)
     */
    private void calculateAndUpdateDistance() {

        double dist = 0;
        if (pose_keeper != null && pose_keeper.size() > 1)
            dist = walking_length(pose_keeper);

        walked_distance = dist;
    }


    /**
     * Calculate the walking length (m)
     */
    private double walking_length(ArrayList<Pose_Structure> points) {
        double distance = 0;
        int size = points.size();
        for (int i = 1; i < size; i++) {
            float v0X = points.get(i - 1).getTrans_X();
            float v1X = points.get(i).getTrans_X();

            float v0Y = points.get(i - 1).getTrans_Y();
            float v1Y = points.get(i).getTrans_Y();

            double temp_dist = 0;
            temp_dist = Math.sqrt(Math.pow((v0X - v1X), 2) + Math.pow((v0Y - v1Y), 2));

            if (temp_dist >= 1) //ignoring big amount of device stability error (mm)
                distance += temp_dist;
        }
        return distance / 100;
    }


    private void startupTango() {
        // Connect listeners to tango service and forward point cloud and camera information to FloorManager.
        mFloorManager = new FloorManager(new FloorManager.OnFloorPlanDataAvailableListener() {
            @Override
            public void onFloorPlanDataAvailable(List<TangoPolygon> polygons) {

                m2D_RenderingView.setFloorplanBoundaryObjects(polygons);
                //calculateAndUpdateArea(polygons);
                // calculateAndUpdateDistance();
            }

        });
        // Set camera intrinsics to FloorManager.
        mFloorManager.setDepthCameraCalibration(mTango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_DEPTH));

        mFloorManager.startFloorplanning();

        List<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();

        //Activate area learning to correct movements drifting
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE));


        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(final TangoPoseData tangoPoseData) {
                //pose is taken from the listener created at onPreDrawing()
                // Passing in the pose data to UX library produce exceptions.


            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData tangoXyzIjData) { //DEPRECATED: This callback is triggered every time a new <<<point cloud>>> is available from the depth sensor in the Tango service. --> use onPointCloudAvailable
                // We are not using onXyzIjAvailable for this app.

            }

            @Override
            public void onFrameAvailable(int cameraId) {

                if(user_allows_openGLView) {
                    // This will get called every time a new RGB camera frame is available to be
                    // rendered.
                   // Log.d(TAG, "onFrameAvailable");

                    if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                        // Now that we are receiving onFrameAvailable callbacks, we can switch
                        // to RENDERMODE_WHEN_DIRTY to drive the render loop from this callback.
                        // This will result on a frame rate of  approximately 30FPS, in synchrony with
                        // the RGB camera driver.
                        // If you need to render at a higher rate (i.e.: if you want to render complex
                        // animations smoothly) you  can use RENDERMODE_CONTINUOUSLY throughout the
                        // application lifecycle.
                        if (mOpenGLSurfaceView.getRenderMode() != GLSurfaceView.RENDERMODE_CONTINUOUSLY) {
                            mOpenGLSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
                        }

                        // Note that the RGB data is not passed as a parameter here.
                        // Instead, this callback indicates that you can call
                        // the {@code updateTexture()} method to have the
                        // RGB data copied directly to the OpenGL texture at the native layer.
                        // Since that call needs to be done from the OpenGL thread, what we do here is
                        // set-up a flag to tell the OpenGL thread to do that in the next run.
                        // NOTE: Even if we are using a render by request method, this flag is still
                        // necessary since the OpenGL thread run requested below is not guaranteed
                        // to run in synchrony with this requesting call.
                        mIsFrameAvailableTangoThread.set(true);
                        // Trigger an OpenGL render to update the OpenGL scene with the new RGB data.
                        mOpenGLSurfaceView.requestRender();

                    }
                }

            }

            @Override
            public void onTangoEvent(TangoEvent tangoEvent) {

                if (mTangoUx != null && user_allows_TangoUx) {
                    mTangoUx.updateTangoEvent(tangoEvent);
                }
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData tangoPointCloudData) {
                if (mTangoUx != null && user_allows_TangoUx) {
                    mTangoUx.updatePointCloud(tangoPointCloudData);
                }
                mFloorManager.onPointCloudAvailable(tangoPointCloudData); //we do not allow direct processing here, but we direct them.
                calculateAndUpdateDistance(); // negligible amount of precessing...

            }
        });
    }



    /**
     * Save the current Area Description File.
     * Performs saving on a background thread and displays a progress dialog.
     */
    private void saveAdf(String adfName) {
        mSaveAdfTask = new SaveAdfTask(this, new SaveAdfTask.SaveAdfListener() {
            @Override
            public void onSaveAdfFailed(String adfName) {
                String toastMessage = "Save ADF faild!" ;

                    showsToastOnUiThread(toastMessage, UIThread_accessibility_providing_activity);
                mSaveAdfTask = null;
            }

            @Override
            public void onSaveAdfSuccess(String adfName, String adfUuid) {

                String toastMessage = "ADF saved!: "+ adfName + ", UUID: "+ adfUuid;

                    showsToastOnUiThread(toastMessage, UIThread_accessibility_providing_activity);
                mSaveAdfTask = null;

            }
        }, mTango, adfName);
        mSaveAdfTask.execute();
    }

    public void onClearButtonClicked(View v) {

        do_reset_all();
    }

    private void do_reset_all() {

        //currFloorNum = startFloorNum;
        if (pose_keeper != null)
            pose_keeper.clear();
        if (floorSwitch_pose_keeper != null)
            floorSwitch_pose_keeper.clear();

        if (mFloorManager != null)
            try {
                mFloorManager.resetFloorplan();
            } catch (Exception ex) {
                Log.i("ERR", "mFloorManager: " + ex.getMessage());
            }

        if (m2D_RenderingView != null)
            try {
                m2D_RenderingView.reset_Map();
            } catch (Exception ex) {
                Log.i("ERR", "m2D_RenderingView: " + ex.getMessage());
            }

        map_scale_seekBar.setProgress(250);


            showsToastOnUiThread(R.string.cleared, UIThread_accessibility_providing_activity);

    }


    public static void showsToastOnUiThread(final int resId, final Activity a) {
        if(a != null)
            a.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(a, a.getString(resId), Toast.LENGTH_SHORT).show();
                }
            });
    }


    public static void showsToastOnUiThread(final String str, final Activity a) {
        if(a!= null)
            a.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(a, str, Toast.LENGTH_LONG).show();
                }
            });
    }


    private void showsToastAndFinishOnUiThread(final int resId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(FloorPlanNavigator.this,
                        getString(resId), Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }

    /**
     * Check we have the necessary permissions for this app.
     */
    private boolean hasCameraPermission() {
        return ContextCompat.checkSelfPermission(this, CAMERA_PERMISSION) == PackageManager.PERMISSION_GRANTED;
    }


    /**
     * Request the necessary permissions for this app.
     */
    private void requestCameraPermission() {
        if (ActivityCompat.shouldShowRequestPermissionRationale(this, CAMERA_PERMISSION)) {
            showRequestPermissionRationale();
        } else {
            ActivityCompat.requestPermissions(this, new String[]{CAMERA_PERMISSION},
                    CAMERA_PERMISSION_CODE);
        }
    }

    /**
     * If the user has declined the permission before, we have to explain him the app needs this
     * permission.
     */
    private void showRequestPermissionRationale() {
        final AlertDialog dialog = new AlertDialog.Builder(this)
                .setMessage("Java Floorplan Reconstruction Example requires camera permission")
                .setPositiveButton("Ok", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        ActivityCompat.requestPermissions(FloorPlanNavigator.this,
                                new String[]{CAMERA_PERMISSION}, CAMERA_PERMISSION_CODE);
                    }
                })
                .create();
        dialog.show();
    }


    /**
     * Check we have the necessary permissions for this app, and ask for them if we haven't.
     *
     * @return True if we have the necessary permissions, false if we haven't.
     */
    private boolean checkAndRequestPermissions() {
        if (!hasCameraPermission()) {
            requestCameraPermission();
            return false;
        }
        return true;
    }


    /**
     * Using com.projecttango.examples.java.hellomotiontracking
     * Log the Position and Orientation of the given pose in the Logcat as information.
     *
     * @param pose the pose to log.
     */
    private void logPose(TangoPoseData pose) {
        StringBuilder stringBuilder = new StringBuilder();

        float translation[] = pose.getTranslationAsFloats();
        stringBuilder.append("Position: " + translation[0] + ", " + translation[1] + ", " + translation[2]);

        float orientation[] = pose.getRotationAsFloats();
        stringBuilder.append(". Orientation: " +
                orientation[0] + ", " + orientation[1] + ", " +
                orientation[2] + ", " + orientation[3]);

       current_device_height = (translation[1] /1) + handHeight; //Y (m)

    }


    private void updateGUI() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //floor output
                mFloor_number.setText(String.valueOf(currFloorNum));

                //avg_depth output
                FloorPlanNavigator.mAvg_depthText.setText(String.format("%.4f", avg_depth));
                if (avg_depth == 0)
                    FloorPlanNavigator.mAvg_depthText.setTextColor(Color.RED);
                else FloorPlanNavigator.mAvg_depthText.setTextColor(Color.GRAY);

                //walked distance output
                final String areaText = String.format("%.3f", walked_distance);

                mDistText.setText(areaText);
                mTVHeight.setText(String.valueOf(String.format("%.3f", current_device_height)));

                textView_rt_height_val.setText(String.valueOf(String.format("%.3f", realTime_height)));

                textView_rt_hand_height_val.setText(String.valueOf(String.format("%.3f", realTime_Hands_height)));

                textView_rest_h_2_switch_val.setText(String.valueOf(String.format("%.3f", height_to_switch)));

                textView_test_scale_val.setText(String.valueOf(String.format("%.3f", m2D_RenderingView.last_test_scale)));

            }
        });
    }


    void make_start_dialog(Activity activity) { //settings dialog

        advanced_dialog = new Dialog(activity);
        advanced_dialog.requestWindowFeature(Window.FEATURE_NO_TITLE);
        advanced_dialog.getWindow().setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
        advanced_dialog.setContentView(R.layout.start_dialog);
        advanced_dialog.setCancelable(true);
        advanced_dialog.setCanceledOnTouchOutside(false);


        final Button btn_hand_height = (Button) advanced_dialog.findViewById(R.id.button_save_hand_height);
        final Button btn_floor_num = (Button) advanced_dialog.findViewById(R.id.button_save_floor_num);

        final LinearLayout LL_floor_height1 = (LinearLayout) advanced_dialog.findViewById(R.id.LL_floor_height_1);
        final LinearLayout LL_floor_height2 = (LinearLayout) advanced_dialog.findViewById(R.id.LL_floor_height_2);
        final LinearLayout LL_floor_height3 = (LinearLayout) advanced_dialog.findViewById(R.id.LL_floor_height_3);
        final LinearLayout LL_floor_height4 = (LinearLayout) advanced_dialog.findViewById(R.id.LL_floor_height_4);

        final Button btn_adjust = (Button) advanced_dialog.findViewById(R.id.button_adjust);
        final Button btn_adjust_neg_1 = (Button) advanced_dialog.findViewById(R.id.button_adjust__1);
        final Button btn_adjust_1 = (Button) advanced_dialog.findViewById(R.id.button_adjust_2);
        final Button btn_adjust_2 = (Button) advanced_dialog.findViewById(R.id.button_adjust_3);
        final Button btn_adjust_3 = (Button) advanced_dialog.findViewById(R.id.button_adjust_4);

        final Button btn_auto_adjust = (Button) advanced_dialog.findViewById(R.id.button_adjust_auto);
        final Button btn_auto_adjust_neg_1 = (Button) advanced_dialog.findViewById(R.id.button_adjust_auto__1);
        final Button btn_auto_adjust_1= (Button) advanced_dialog.findViewById(R.id.button_adjust_auto_2);
        final Button btn_auto_adjust_2 = (Button) advanced_dialog.findViewById(R.id.button_adjust_auto_3);
        final Button btn_auto_adjust_3 = (Button) advanced_dialog.findViewById(R.id.button_adjust_auto_4);

        final Button btn_start = (Button) advanced_dialog.findViewById(R.id.button_start);
        final Button btn_mng_adfs = (Button) advanced_dialog.findViewById(R.id.button_mng_ADFs);
        final Button btn_save_adf = (Button) advanced_dialog.findViewById(R.id.button_save_ADF);
        final Button btn_init_mapScale_factor = (Button) advanced_dialog.findViewById(R.id.button_save_init_mapScale_factor);

        final SeekBar depth_seekBar = (SeekBar) advanced_dialog.findViewById(R.id.seekBar_far);

        final TextView mbs = (TextView) advanced_dialog.findViewById(R.id.textView_Tango_status);
        final TextView tv_far_val = (TextView) advanced_dialog.findViewById(R.id.textView_far_val);

        final EditText et_hand_height = (EditText) advanced_dialog.findViewById(R.id.editText_hand_height);
        final EditText et_floor_num = (EditText) advanced_dialog.findViewById(R.id.editText_floor_num);
        final EditText et_init_mapScale_factor = (EditText) advanced_dialog.findViewById(R.id.editText_init_mapScale_factor);

        final EditText et_floor_height = (EditText) advanced_dialog.findViewById(R.id.editText_Floor_Height);
        final EditText et_floor_height_neg_1 = (EditText) advanced_dialog.findViewById(R.id.editText_Floor_Height__1);
        final EditText et_floor_height_1 = (EditText) advanced_dialog.findViewById(R.id.editText_Floor_Height_2);
        final EditText et_floor_height_2 = (EditText) advanced_dialog.findViewById(R.id.editText_Floor_Height_3);
        final EditText et_floor_height_3 = (EditText) advanced_dialog.findViewById(R.id.editText_Floor_Height_4);

        final EditText et_ADF_name = (EditText) advanced_dialog.findViewById(R.id.editText_ADF_name);

        final CheckBox cb_unified_f_height = (CheckBox) advanced_dialog.findViewById(R.id.checkBox_unified_f_height);
        final CheckBox cb_grid = (CheckBox) advanced_dialog.findViewById(R.id.checkBox_grid);
        final CheckBox cb_vibrate = (CheckBox) advanced_dialog.findViewById(R.id.checkBox_vibration);
        final CheckBox cb_opengl_View = (CheckBox) advanced_dialog.findViewById(R.id.checkBox_opengl);
        final CheckBox cb_tango_ux = (CheckBox) advanced_dialog.findViewById(R.id.checkBox_tangoUX);
        final CheckBox cb_areaLearning = (CheckBox) advanced_dialog.findViewById(R.id.checkBox_areaLearningMode);

        if (service_has_started) {
            btn_start.setText("Restart Service");

            WindowManager.LayoutParams WMLP = advanced_dialog.getWindow().getAttributes();

            WMLP.x = 650;   //x position
            WMLP.y = -50;   //y position

            advanced_dialog.getWindow().setAttributes(WMLP);

            advanced_dialog.setCanceledOnTouchOutside(true);
        }

        String msg = "";
        if (mTango != null && mIsConnected)
            msg = "Connected";
        else
            msg = "Disconnectd";
        mbs.setText(msg);

        if(!different_floors_heights)
        {
            LL_floor_height1.setVisibility(View.GONE);
            LL_floor_height2.setVisibility(View.GONE);
            LL_floor_height3.setVisibility(View.GONE);
            LL_floor_height4.setVisibility(View.GONE);
        }
        else
        {
            LL_floor_height1.setVisibility(View.VISIBLE);
            LL_floor_height2.setVisibility(View.VISIBLE);
            LL_floor_height3.setVisibility(View.VISIBLE);
            LL_floor_height4.setVisibility(View.VISIBLE);

            et_floor_height.setText(String.format(Locale.ROOT, "%.2f", estimated_floor_heightS[0]));
            et_floor_height_neg_1.setText(String.format(Locale.ROOT, "%.2f", estimated_floor_heightS[1]));
            et_floor_height_1.setText(String.format(Locale.ROOT, "%.2f", estimated_floor_heightS[2]));
            et_floor_height_2.setText(String.format(Locale.ROOT, "%.2f", estimated_floor_heightS[3]));
            et_floor_height_3.setText(String.format(Locale.ROOT, "%.2f", estimated_floor_heightS[4]));
        }

        if (user_allows_openGLView) {
            cb_opengl_View.setChecked(true);
            openGLSurfaceFrameLayout.setVisibility(View.VISIBLE);
            if(mOpenGLSurfaceView != null)
                mOpenGLSurfaceView.setVisibility(View.VISIBLE);
        }
        else {
            cb_opengl_View.setChecked(false);
            openGLSurfaceFrameLayout.setVisibility(View.GONE);
            if(mOpenGLSurfaceView != null)
                mOpenGLSurfaceView.setVisibility(View.GONE);
        }

        cb_vibrate.setChecked(user_allows_vibration);
        cb_grid.setChecked(show_grid);
        cb_tango_ux.setChecked(user_allows_TangoUx);
        cb_areaLearning.setChecked(user_allows_DriftCorrection);
        cb_unified_f_height.setChecked(!different_floors_heights);

        et_init_mapScale_factor.setText(String.format(Locale.ROOT, "%.3f", init_scale_factor));
        et_floor_num.setText(String.valueOf(startFloorNum));
        et_hand_height.setText(String.format(Locale.ROOT, "%.2f", handHeight));
        et_floor_height.setText(String.format(Locale.ROOT, "%.2f", floorHeight));

        tv_far_val.setText(String.valueOf(limit_far));
        depth_seekBar.setProgress((int) limit_far * 100);
        depth_seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                limit_far = Double.valueOf(i / 100.0);
                tv_far_val.setText(String.valueOf(limit_far));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                //do nothing
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

                //Store prefs:
                SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                SharedPreferences.Editor editor = settings.edit();
                editor.putFloat(PREFS_FAR, (float) limit_far);
                editor.apply();

            }
        });


        cb_grid.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                               @Override
                                               public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                   if (isChecked)
                                                       show_grid = true;
                                                   else show_grid = false;

                                                   //Store prefs:
                                                   SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                                   SharedPreferences.Editor editor = settings.edit();
                                                   editor.putBoolean(PREFS_GRID_VISIBILITY, show_grid);
                                                   editor.apply();

                                               }
                                           }
        );

        cb_unified_f_height.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                               @Override
                                               public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                   if (isChecked)
                                                       different_floors_heights = false;
                                                   else different_floors_heights = true;

                                                   //Store prefs:
                                                   SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                                   SharedPreferences.Editor editor = settings.edit();
                                                   editor.putBoolean(PREFS_UNI_HEIGHTS, different_floors_heights);
                                                   editor.apply();

                                               }
                                           }
        );

        cb_areaLearning.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                                       @Override
                                                       public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                           if (isChecked) {
                                                               user_allows_DriftCorrection = true;
                                                               isDriftCorrection = true;
                                                               isLearningMode = false;
                                                               isLoadAdf = false;
                                                               allow_update_cam_poses = false;
                                                           }
                                                           else {
                                                               user_allows_DriftCorrection = false;
                                                               isDriftCorrection = false;
                                                               isLearningMode = true; //after a fresh start (not based on ADF) can save ADF. By starting based on loaded ADF, a new ADF can only be saved after localized.
                                                               isLoadAdf = true; //will load an ADF if found and calibrate it
                                                               allow_update_cam_poses = false;
                                                           }

                                                           //Store prefs:
                                                           SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                                           SharedPreferences.Editor editor = settings.edit();
                                                           editor.putBoolean(PREFS_AREA_LEARNING, user_allows_DriftCorrection);
                                                           editor.apply();

                                                       }
                                                   }
        );

        cb_vibrate.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                                  @Override
                                                  public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                      if (isChecked)
                                                          user_allows_vibration = true;
                                                      else user_allows_vibration = false;

                                                      //Store prefs:
                                                      SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                                      SharedPreferences.Editor editor = settings.edit();
                                                      editor.putBoolean(PREFS_VIBRATION, user_allows_vibration);
                                                      editor.apply();
                                                  }
                                              }
        );

        cb_opengl_View.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                                      @Override
                                                      public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                          if (isChecked) {
                                                              user_allows_openGLView = true;
                                                              openGLSurfaceFrameLayout.setVisibility(View.VISIBLE);
                                                              if(mOpenGLSurfaceView != null)
                                                                mOpenGLSurfaceView.setVisibility(View.VISIBLE);
                                                          }
                                                          else {
                                                              user_allows_openGLView = false;
                                                              openGLSurfaceFrameLayout.setVisibility(View.GONE);
                                                              if(mOpenGLSurfaceView != null)
                                                                mOpenGLSurfaceView.setVisibility(View.GONE);
                                                          }

                                                          //Store prefs:
                                                          SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                                          SharedPreferences.Editor editor = settings.edit();
                                                          editor.putBoolean(PREFS_OPENGL_VIEW, user_allows_openGLView);
                                                          editor.apply();
                                                      }
                                                  }
        );


        cb_tango_ux.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                                      @Override
                                                      public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                          if (isChecked) {
                                                              user_allows_TangoUx = true;
                                                          }
                                                          else {
                                                              user_allows_TangoUx = false;
                                                          }

                                                          //Store prefs:
                                                          SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                                          SharedPreferences.Editor editor = settings.edit();
                                                          editor.putBoolean(PREFS_TANGO_UX, user_allows_TangoUx);
                                                          editor.apply();
                                                      }
                                                  }
        );


        btn_hand_height.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {


                    if (!et_hand_height.getText().toString().equals("")) {
                        handHeight = Float.valueOf(et_hand_height.getText().toString());

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putFloat(PREFS_PERSON_HAND_HEIGHT, (float) handHeight);
                        editor.apply();

                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                        //here you have to restart all!
                        try {
                            do_reset_all();
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });


        btn_mng_adfs.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                advanced_dialog.dismiss();
                startAdfListView();
            }
        });

        btn_save_adf.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if(service_has_started && isLearningMode) //not isDriftCorrection
                {
                    advanced_dialog.dismiss();
                    if(!et_ADF_name.getText().toString().equals(""))
                        saveAdf(et_ADF_name.getText().toString());
                    else
                    {
                        DateFormat df = new SimpleDateFormat("yyMMddHHmmssZ");
                        String date = df.format(Calendar.getInstance().getTime());
                        saveAdf("ADF:" + date);
                    }
                }
                else
                {
                    String toastMessage = "You can't save an ADF using this mode! Use Learning Mode instead (Disable Drift Corr.)" ;
                    showsToastOnUiThread(toastMessage, UIThread_accessibility_providing_activity);
                }

            }
        });


        btn_floor_num.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {


                    if (!et_floor_num.getText().toString().equals("")) {
                        startFloorNum = Integer.valueOf(et_floor_num.getText().toString());
                        currFloorNum = startFloorNum;

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putInt(PREFS_START_FLOOR, startFloorNum);
                        editor.apply();

                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                        //here you have to restart all!
                        try {
                            do_reset_all();
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });


        btn_init_mapScale_factor.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {


                    if (!et_init_mapScale_factor.getText().toString().equals("")) {
                        init_scale_factor = Float.valueOf(et_init_mapScale_factor.getText().toString());

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putFloat(PREFS_INIT_SCALE_FACTOR, init_scale_factor);
                        editor.apply();

                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });


        btn_adjust.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {
                    if(!different_floors_heights) {
                        if (!et_floor_height.getText().toString().equals("")) {
                            floorHeight = Double.valueOf(et_floor_height.getText().toString());

                            //Store prefs:
                            SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                            SharedPreferences.Editor editor = settings.edit();
                            editor.putFloat(PREFS_FLOOR_HEIGHT, (float) floorHeight);
                            editor.apply();

                            height_auto_adjustment_mode = false;
                            showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                            //here you have to restart all!
                            try {
                                do_reset_all();
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                    }
                    else
                    {
                        if (!et_floor_height.getText().toString().equals("")) {
                            estimated_floor_heightS[1] = Double.valueOf(et_floor_height.getText().toString());

                            //Store prefs:
                            SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                            SharedPreferences.Editor editor = settings.edit();
                            editor.putFloat(PREFS_FLOOR_HEIGHT, (float) estimated_floor_heightS[1]);
                            editor.apply();

                            height_auto_adjustment_mode = false;
                            showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                            //here you have to restart all!
                            try {
                                do_reset_all();
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });

        btn_adjust_neg_1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {
                    if (!et_floor_height_neg_1.getText().toString().equals("")) {
                        estimated_floor_heightS[0] = Double.valueOf(et_floor_height_neg_1.getText().toString());

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putFloat(PREFS_FLOOR_HEIGHT2, (float) estimated_floor_heightS[0]);
                        editor.apply();

                        height_auto_adjustment_mode = false;
                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                        //here you have to restart all!
                        try {
                            do_reset_all();
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });

        btn_adjust_1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {
                    if (!et_floor_height_1.getText().toString().equals("")) {
                        estimated_floor_heightS[2] = Double.valueOf(et_floor_height_1.getText().toString());

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putFloat(PREFS_FLOOR_HEIGHT3, (float) estimated_floor_heightS[2]);
                        editor.apply();

                        height_auto_adjustment_mode = false;
                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                        //here you have to restart all!
                        try {
                            do_reset_all();
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });

        btn_adjust_2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {
                    if (!et_floor_height_2.getText().toString().equals("")) {
                        estimated_floor_heightS[3] = Double.valueOf(et_floor_height_2.getText().toString());

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putFloat(PREFS_FLOOR_HEIGHT4, (float) estimated_floor_heightS[3]);
                        editor.apply();

                        height_auto_adjustment_mode = false;
                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                        //here you have to restart all!
                        try {
                            do_reset_all();
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });

        btn_adjust_3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                try {
                    if (!et_floor_height_3.getText().toString().equals("")) {
                        estimated_floor_heightS[4] = Double.valueOf(et_floor_height_3.getText().toString());

                        //Store prefs:
                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                        SharedPreferences.Editor editor = settings.edit();
                        editor.putFloat(PREFS_FLOOR_HEIGHT5, (float) estimated_floor_heightS[4]);
                        editor.apply();

                        height_auto_adjustment_mode = false;
                        showsToastOnUiThread("Saved.", UIThread_accessibility_providing_activity);

                        //here you have to restart all!
                        try {
                            do_reset_all();
                        } catch (Exception ex) {
                            ex.printStackTrace();
                        }
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                    showsToastOnUiThread("Bad Input!", UIThread_accessibility_providing_activity);
                }
            }
        });

        btn_auto_adjust.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (advanced_dialog != null && advanced_dialog.isShowing())
                    advanced_dialog.dismiss();

                if(mTangoUx != null && user_allows_TangoUx)
                    mTangoUx.setHoldPosture(TYPE_HOLD_POSTURE_UP); //TYPE_HOLD_POSTURE_UP

                if(!different_floors_heights)
                    adjust_floor_height_automatically(-1);
                else adjust_floor_height_automatically(0);
            }
        });

        btn_auto_adjust_neg_1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (advanced_dialog != null && advanced_dialog.isShowing())
                    advanced_dialog.dismiss();

                if(mTangoUx != null && user_allows_TangoUx)
                    mTangoUx.setHoldPosture(TYPE_HOLD_POSTURE_UP); //TYPE_HOLD_POSTURE_UP

                adjust_floor_height_automatically(1);
            }
        });

        btn_auto_adjust_1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (advanced_dialog != null && advanced_dialog.isShowing())
                    advanced_dialog.dismiss();

                if(mTangoUx != null && user_allows_TangoUx)
                    mTangoUx.setHoldPosture(TYPE_HOLD_POSTURE_UP); //TYPE_HOLD_POSTURE_UP

                adjust_floor_height_automatically(2);
            }
        });

        btn_auto_adjust_2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (advanced_dialog != null && advanced_dialog.isShowing())
                    advanced_dialog.dismiss();

                if(mTangoUx != null && user_allows_TangoUx)
                    mTangoUx.setHoldPosture(TYPE_HOLD_POSTURE_UP); //TYPE_HOLD_POSTURE_UP

                adjust_floor_height_automatically(3);
            }
        });

        btn_auto_adjust_3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (advanced_dialog != null && advanced_dialog.isShowing())
                    advanced_dialog.dismiss();

                if(mTangoUx != null && user_allows_TangoUx)
                    mTangoUx.setHoldPosture(TYPE_HOLD_POSTURE_UP); //TYPE_HOLD_POSTURE_UP

                adjust_floor_height_automatically(4);
            }
        });


        btn_start.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (service_has_started) {

                    do_stop_Tango_service();
                }

                service_has_started = true;
                start_tango();
                do_onStart();

                m2D_RenderingView.mIsDrawing = true;
                advanced_dialog.dismiss();
                ((Button) v).setText("Restart Service");
                //v.setEnabled(false);
                //((Button) v).setTextColor(Color.GRAY);
            }
        });

        getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_HIDDEN);
        advanced_dialog.show();

    }

    private void start_tango() {

        pose_keeper = new ArrayList<>();
        floorSwitch_pose_keeper = new ArrayList<>();
        m2D_RenderingView = (_2D_RenderingView) findViewById(R.id.floorplan);
        m2D_RenderingView.registerCallback(this);
        mDistText = (TextView) findViewById(R.id.dist_text);
        mAvg_depthText = (TextView) findViewById(R.id.avg_depth_text);
        mFloor_number = (TextView) findViewById(R.id.textView_floor_num);
        mFloor_number.setText(String.valueOf(startFloorNum));
        //if(user_allows_TangoUx)
            mTangoUx = setupTangoUxAndLayout();
        mTVHeight= (TextView) findViewById(R.id.textView_height_val);
        textView_rt_height_val= (TextView) findViewById(R.id.textView_rt_height_val);
        textView_rt_hand_height_val = (TextView) findViewById(R.id.textView_RT_HandH_val);
        textView_rest_h_2_switch_val = (TextView) findViewById(R.id.textView_rest_h_2_switch_val);
        textView_test_scale_val = (TextView) findViewById(R.id.textView_TestS_val);
        cb_map_mode = (CheckBox) findViewById(R.id.checkBox_map_mode);
        cb_sclae_test_mode = (CheckBox) findViewById(R.id.checkBox_testScale);

        cb_map_mode.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                               @Override
                                               public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                   if (isChecked)
                                                       user_allows_Map = true;
                                                   else user_allows_Map = false;

                                               }
                                           }
        );

        cb_sclae_test_mode.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                                   @Override
                                                   public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                                                       if (isChecked ) {
                                                           if(FloorManager.camera_corner_pos_initialized) {
                                                               user_allows_Scale_test = true;
                                                           }
                                                           else cb_sclae_test_mode.setChecked(false);
                                                       }
                                                       else {
                                                           user_allows_Scale_test = false;
                                                       }

                                                   }
                                               }
        );


        mTVLocalization= (TextView) findViewById(R.id.textView_localization_val);

        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE); //to handle screen rotation events (Portrait - Landscape)
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {
                    //do nothing
                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setDisplayRotation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {
                    //do nothing
                }
            }, null);
        }

    }


    /**
     * Sets up TangoUX layout and sets its listener.
     */
    private TangoUx setupTangoUxAndLayout() {

        TangoUxLayout uxLayout = (TangoUxLayout) findViewById(R.id.layout_tango);
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setLayout(uxLayout);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);

        //if(user_allows_TangoUx)
            return tangoUx;
        //else return null;
    }


    /*
  * This is an advanced way of using UX exceptions. In most cases developers can just use the in
  * built exception notifications using the Ux Exception layout. In case a developer doesn't want
  * to use the default Ux Exception notifications, he can set the UxException listener as shown
  * below.
  * In this example we are just logging all the ux exceptions to logcat, but in a real app,
  * developers should use these exceptions to contextually notify the user and help direct the
  * user in using the device in a way Tango service expects it.
  */
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {

        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {

            if(user_allows_TangoUx) {
                if (uxExceptionEvent.getType() == TYPE_LYING_ON_SURFACE) {
                    Log.i(TAG, "Device lying on surface ");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                    Log.i(TAG, "Very few depth points in mPoint cloud ");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                    Log.i(TAG, "Invalid poses in MotionTracking ");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_INCOMPATIBLE_VM) {
                    Log.i(TAG, "Device not running on ART");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                    Log.i(TAG, "Invalid poses in MotionTracking ");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                    Log.i(TAG, "Invalid poses in MotionTracking ");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_OVER_EXPOSED) {
                    Log.i(TAG, "Fisheye Camera Over Exposed");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_UNDER_EXPOSED) {
                    Log.i(TAG, "Fisheye Camera Under Exposed ");
                }
                if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_TANGO_SERVICE_NOT_RESPONDING) {
                    Log.i(TAG, "TangoService is not responding ");
                }
                if (uxExceptionEvent.getType() == TYPE_HOLD_POSTURE_UP) {
                    Log.i(TAG, "Device is towards sky ");
                }
            }
        }
    };


    private static int start_automatically_timer = 10; //seconds for the next function
    private static ToastHelper last_count_toast = null;

    private void adjust_floor_height_automatically(final int floor) { // starts in thread //floor = -1 same height, else floor num

        if (service_has_started) {
            last_count_toast = new ToastHelper(UIThread_accessibility_providing_activity);

            start_automatically_timer = 10;

            Thread thread = new Thread() { //remove plitz
                @Override
                public void run() {

                    while (start_automatically_timer > 0) {
                        try {
                            sleep(1000); //one second per tick
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        if (UIThread_accessibility_providing_activity != null)
                            UIThread_accessibility_providing_activity.runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    if (UIThread_accessibility_providing_activity != null) {

                                        last_count_toast.makeText(UIThread_accessibility_providing_activity, "Adjusting in " + start_automatically_timer, 1000).show();
                                    }
                                }
                            });

                        start_automatically_timer--;
                    }

                    if (UIThread_accessibility_providing_activity != null)
                        UIThread_accessibility_providing_activity.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {

                                if(avg_depth >= handHeight) { //registered person height minimum

                                    height_auto_adjustment_mode = true;

                                    if(floor == -1) {
                                        floorHeight = avg_depth;
                                        //Store prefs:
                                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                        SharedPreferences.Editor editor = settings.edit();
                                        editor.putFloat(PREFS_FLOOR_HEIGHT, (float) floorHeight);
                                        editor.apply();
                                    }
                                    else
                                    {
                                            estimated_floor_heightS[floor] = avg_depth;

                                        //Store prefs:
                                        SharedPreferences settings = UIThread_accessibility_providing_activity.getApplicationContext().getSharedPreferences(PREFS_NAME, 0);
                                        SharedPreferences.Editor editor = settings.edit();
                                        if(floor == 0) {
                                            editor.putFloat(PREFS_FLOOR_HEIGHT1, (float) avg_depth);
                                            editor.apply();
                                        }
                                        else  if(floor == 1) {
                                            editor.putFloat(PREFS_FLOOR_HEIGHT2, (float) avg_depth);
                                            editor.apply();
                                        }
                                        else  if(floor == 2) {
                                            editor.putFloat(PREFS_FLOOR_HEIGHT3, (float) avg_depth);
                                            editor.apply();
                                        }
                                        else  if(floor == 3) {
                                            editor.putFloat(PREFS_FLOOR_HEIGHT4, (float) avg_depth);
                                            editor.apply();
                                        }
                                        else  if(floor == 4) {
                                            editor.putFloat(PREFS_FLOOR_HEIGHT5, (float) avg_depth);
                                            editor.apply();
                                        }
                                    }



                                    if (UIThread_accessibility_providing_activity != null) {

                                        last_count_toast.makeText(UIThread_accessibility_providing_activity, "Adjusted to " + String.format(Locale.ROOT, "%.2f", avg_depth) + " m", 4000).show();

                                        height_manually_adjusted = true;

                                        Android_Functionality_Class.make_robot_sound(UIThread_accessibility_providing_activity);

                                        make_start_dialog(UIThread_accessibility_providing_activity);
                                    }
                                }
                                else {
                                    height_auto_adjustment_mode = false;
                                    last_count_toast.makeText(UIThread_accessibility_providing_activity, "Adjustment failed! Height value stays: " + String.format(Locale.ROOT, "%.2f", avg_depth) + " m", 4000).show();
                                }

                                if(mTangoUx != null && user_allows_TangoUx)
                                    mTangoUx.setHoldPosture(TYPE_HOLD_POSTURE_NONE);

                            }
                        });


                    //here you have to restart all!
                    try {
                        do_reset_all();
                    } catch (Exception ex) {
                        ex.printStackTrace();
                    }
                }
            };

            thread.start();
        } else {

                showsToastOnUiThread("Automatic adjustment works only when Tango Service is connected!", UIThread_accessibility_providing_activity);
        }


    }


    public void onSettings_btn_clicked(View v) {
        make_start_dialog(UIThread_accessibility_providing_activity);
    }

    void do_onStart() {
        synchronized (this) {
            if (service_has_started) {
                // Check and request camera permission at run time.
                if (checkAndRequestPermissions()) {
                    bindTangoService();
                    do_onResume_Vid();
                }
            }
        }

        Log.i("do_onStart", "Started!");
    }


    //for Video
    /**
     * Here is where to set-up rendering logic. We're replacing it with a minimalistic,
     * dummy example using a standard GLSurfaceView and a basic renderer, for illustration purposes only.
     */


    static boolean OpenGL_View_default_size = true;
    static RelativeLayout.LayoutParams vid_default_size_params;

    private void setupVideoRenderer() {

        mOpenGLSurfaceView = new GLSurfaceView(getApplication());
        //mOpenGLSurfaceView = (GLSurfaceView) findViewById(R.id.video_surfaceview2);
        OpenGL_View_default_size = true;
        vid_default_size_params = (RelativeLayout.LayoutParams) openGLSurfaceFrameLayout.getLayoutParams();
        mOpenGLSurfaceView.setOnTouchListener(new OnSwipeTouchListener() {
            @Override
            public void onRightToLeft() {
                if(OpenGL_View_default_size) {

                    RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams) openGLSurfaceFrameLayout.getLayoutParams();
                    params.removeRule(RelativeLayout.ALIGN_PARENT_RIGHT);
                    params.addRule(RelativeLayout.ALIGN_PARENT_LEFT);
                    openGLSurfaceFrameLayout.setLayoutParams(params);
                }
            }

            @Override
            public void onLeftToRight() {
                if(OpenGL_View_default_size) {

                    RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams) openGLSurfaceFrameLayout.getLayoutParams();
                    params.removeRule(RelativeLayout.ALIGN_PARENT_LEFT);
                    params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
                    openGLSurfaceFrameLayout.setLayoutParams(params);
                }
            }

            @Override
            public void onBottomToTop() {
                //do nothing
            }

            @Override
            public void onTopToBottom() {
                //do nothing
            }

            @Override
            public void click() {
                if(OpenGL_View_default_size) {
                    openGLSurfaceFrameLayout.setLayoutParams(new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
                    OpenGL_View_default_size = false;
                }
                else
                {
                    openGLSurfaceFrameLayout.setLayoutParams(vid_default_size_params);
                    OpenGL_View_default_size = true;
                }
            }
        });

        mOpenGLSurfaceView.setEGLContextClientVersion(2);

        mOpenGL_Renderer = new Video_Renderer(new Video_Renderer.RenderCallback() {
            @Override
            public void preRender() {

                if (service_has_started && user_allows_openGLView) {
                    //Log.d(TAG, "preRender Vid: " + mIsConnected);
                    // This is the work that you would do on your main OpenGL render thread.
                    // We need to be careful to not run any Tango-dependent code in the OpenGL
                    // thread unless we know the Tango service to be properly set-up and connected.
                    if (!mIsConnected) {
                        return;
                    }

                    try {
                        // Synchronize against concurrently disconnecting the service triggered from the
                        // UI thread.
                        synchronized (FloorPlanNavigator.this) {
                            // Connect the Tango SDK to the OpenGL texture ID where we are going to
                            // render the camera.
                            // NOTE: This must be done after both the texture is generated and the Tango
                            // service is connected.
                            if (mConnectedTextureIdGlThread == INVALID_TEXTURE_ID) {
                                mConnectedTextureIdGlThread = mOpenGL_Renderer.getTextureId();
                                mTango.connectTextureId(TangoCameraIntrinsics.TANGO_CAMERA_COLOR, mOpenGL_Renderer.getTextureId());
                                Log.i(TAG, "connected to texture id: " + mOpenGL_Renderer.getTextureId());
                            }

                            // If there is a new RGB camera frame available, update the texture and
                            // scene camera pose.
                            if (mIsFrameAvailableTangoThread.compareAndSet(true, false)) {
                                double rgbTimestamp = mTango.updateTexture(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
                                // {@code rgbTimestamp} contains the exact timestamp at which the
                                // rendered RGB frame was acquired.

                                // In order to see more details on how to use this timestamp to modify
                                // the scene camera and achieve an augmented reality effect, please
                                // refer to java_augmented_reality_example and/or
                                // java_augmented_reality_opengl_example projects.

                                // Log and display timestamp for informational purposes
                                //Log.d(TAG, "Frame updated. Timestamp: " + rgbTimestamp);

                                // Updating the UI needs to be in a separate thread. Do it through a
                                // final local variable to avoid concurrency issues.
                                // final String timestampText = String.format(sTimestampFormat,
                                //        rgbTimestamp);
                                // runOnUiThread(new Runnable() {
                                //     @Override
                                //    public void run() {
                                //         mTimestampTextView.setText(timestampText);
                                //     }
                                // });
                            }

                        }
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Tango API call error within the OpenGL thread", e);
                    } catch (Throwable t) {
                        Log.e(TAG, "Exception on the OpenGL thread", t);
                    }
                }
            }
        });
        mOpenGLSurfaceView.setRenderer(mOpenGL_Renderer);
        openGLSurfaceFrameLayout.addView(mOpenGLSurfaceView);
        mOpenGLSurfaceView.setZOrderMediaOverlay(true);
    }


    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // The result of the permission activity.
        // Note that when the permission activity is dismissed, the HelloAreaDescriptionActivity's
        // onResume() callback is called. As the TangoService is connected in the onResume()
        // function, we do not call connect here.
        // Check which request we're responding to
        if (requestCode == REQUEST_CODE_TANGO_PERMISSION) {
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, "Permission granted", Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }


    /**
     * Start the ADF list activity.
     */
    private void startAdfListView() {
        Intent startAdfListViewIntent = new Intent(this, AdfUuidListViewActivity.class);
        startActivity(startAdfListViewIntent);
    }


    //OpenCv:
    //Android calls the "onCreate" method before loading the OpenCV4Android library -->
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i("OpenCV", "OpenCV loaded successfully");
                    OpenCv_loaded = true;

                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };


}
