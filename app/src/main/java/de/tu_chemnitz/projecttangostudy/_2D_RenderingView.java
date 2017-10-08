package de.tu_chemnitz.projecttangostudy;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import com.google.atap.tango.reconstruction.TangoPolygon;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Mat.zeros;

/**
 * Created by Bashar Al Halabi on 12.04.2017.
 * Idea: from com.projecttango.examples.java.floorplanreconstruction
 * Note: Calculation style does not maintain restrictions of solid OOP principles intentionally at the time being for experimental purposes
 */

public class _2D_RenderingView extends SurfaceView implements SurfaceHolder.Callback, OnFloorSwitchListener {

    private static final String TAG = _2D_RenderingView.class.getSimpleName();

    //Global variables group ----------------------------------------------------------------------------------

    Bitmap floorplan_image = null;

    //IP vars:
    Point corner_point_after_IP = null;
    Point corner_point_right_after_IP = null;
    Point corner_point_left_after_IP = null;
    Bitmap resized_IP_Bitmap = null; //patch
    Bitmap Corners_detected_Bitmap = null;
    int IP_patch_size = 150;
    //-------

    Path init_orientation_path = null;
    float[] virtual_start_direction_line = null; //points of the last path "init_orientation_path" to show a line parallel to view field centre in front of cam at start

    int cornerCase_byAddedVirtualPoints = 0; //1, 2, 3 ,4 (4 cases)

    Path triangular_ctrl_points_container_path = null; //the yellow triangle defining the target corner

    //input virt points by user:
    float[] p1_static = new float[2];
    float[] p2_static = new float[2];
    float[] p3_static = new float[2];
    //--------------------

    float angle_btwn_vrtl_real_corners = 0.0f;
    float[] virtual_point_middle_of_p1_and_p3 = new float[2];
    float[] last_calculated_real_points = new float[6]; //3* (x,y)

    static float translationX = 0, translationY = 0; //to translate plan image to meet centre of screen
    static float translationX_map = 0; //..helping
    static float translationY_map = 0; //..helping

    private static final float SCALE = 100f; //virtual meter: Scale between meters and pixels. Hardcoded to a reasonable default. (1 m = 100 pixels)
    private static double pitch = 0; //to simulate cam-view up-down moves

    private volatile List<TangoPolygon> mPolygons = new ArrayList<>(); //registered Tango polygons

    //different used colors:
    private Paint mTopTrianglePaint;
    private Paint mWallPaint;
    private Paint mFurniturePaint;
    private Paint mSpacePaint;
    private Paint mUserMarkerPaint;
    private Paint mPathPaint_currentFloor;
    private Paint mPathPaint_upperFloor;
    private Paint mPathPaint_FloorBelow;
    private Paint mDirectPathPaint;
    //------

    private Path mTopTrianglePath;
    private Path mUserMarkerPath;
    private Path mBashPath;
    private Path mDirectPath;

    private Matrix mCamera; //camera poses calculated through Tango
    private Matrix mCameraInverse;
    private Matrix matrix_calibrated = new Matrix();

    private SurfaceHolder mSurfaceHolder;

    boolean mIsDrawing = false;
    private static boolean mIsFloorSwitchingMode = false;
    private static FloorSwitch mFloorSwitchType = FloorSwitch.no_switch;
    public static float last_scale = 2.5f;
    public static float last_test_scale = 2.5f; //only for calibration of the bitmap (floor plan) size

    private float[] stored_old_val_of_head_corner = null; //temp. buffer
    private boolean allow_update_cam_poses = false; //for init. purposes
    private float[] virtual_projection_of_head_on_line_LeftRight_calibrated_pt; //global only to visualize later
    private float init_angle_fix = Float.MAX_VALUE;
    private float[] init_pose_fix = null;   //a helping point to hold calibration info. for the next draw

    //testing vars:
    float[] testing_after_calibrating_real_points;
    private float[] testing_after_calib_virt_points;
    private float[] the_manuell_added_virtual_right_point = null;
    private float[] temp_point_show;
    static float rotation_of_map_init = 0;
    private float cam_rot;
    private Path mOcclusionPath;
    float[] tmp_pt = new float[2];


    //End of global variables group ---------------------------------------------------


    /**
     * Handle switching between floors
     */
    @Override
    public void OnFloorSwitch(FloorObject.FloorSwitchEvent event, int currentFloor) {

        //inform about floor switch

        FloorPlanNavigator.showsToastOnUiThread(event.toString() + ": " + currentFloor, FloorPlanNavigator.UIThread_accessibility_providing_activity);

        //store the floor switching coordinates to trace later
        mIsFloorSwitchingMode = true;
        mFloorSwitchType = event.switchType();

        if (currentFloor == 0)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage0);
        else if (currentFloor == 1)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage1);
        else if (currentFloor == 2)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage2);
        else if (currentFloor == 3)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage3);
    }

    /**
     * Custom render thread, running at a fixed 100Hz rate.
     */
    private class RenderThread extends Thread {
        @Override
        public void run() {
            while (mIsDrawing) {
                Canvas canvas = mSurfaceHolder.lockCanvas();
                if (canvas != null) {
                    doDraw(canvas);
                    mSurfaceHolder.unlockCanvasAndPost(canvas);
                }
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                }
            }
        }
    }

    ;
    private RenderThread mDrawThread;

    /**
     * Pre drawing callback.
     */
    public interface DrawingCallback {
        /**
         * Called during onDraw, before any element is drawn to the view canvas.
         */
        void onPreDrawing();
    }

    private DrawingCallback mCallback;

    public _2D_RenderingView(Context context) {
        super(context);
        init();
    }

    public _2D_RenderingView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public _2D_RenderingView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {

        for (int i = 0; i < 6; i++)
            last_calculated_real_points[i] = 0;

        // Pre-create graphics objects.
        added_points = new ArrayList<>();

        mWallPaint = new Paint();
        mWallPaint.setColor(getResources().getColor(android.R.color.black));
        mWallPaint.setStyle(Paint.Style.STROKE);
        mWallPaint.setStrokeWidth(3);
        mSpacePaint = new Paint();
        mSpacePaint.setColor(Color.CYAN);
        mSpacePaint.setStyle(Paint.Style.FILL);
        mFurniturePaint = new Paint();
        mFurniturePaint.setColor(Color.YELLOW);
        mFurniturePaint.setStyle(Paint.Style.FILL);
        createUserMarkerShape(pitch);

        mOcclusionPath = new Path();

        mBashPath = new Path();

        mDirectPath = new Path();

        mPathPaint_currentFloor = new Paint(Paint.ANTI_ALIAS_FLAG);
        mPathPaint_currentFloor.setStyle(Paint.Style.STROKE);
        mPathPaint_currentFloor.setStrokeWidth(4);
        mPathPaint_currentFloor.setColor(Color.BLUE);
        mPathPaint_currentFloor.setAlpha(90);

        mPathPaint_upperFloor = new Paint(Paint.ANTI_ALIAS_FLAG);
        mPathPaint_upperFloor.setStyle(Paint.Style.STROKE);
        mPathPaint_upperFloor.setStrokeWidth(10);
        mPathPaint_upperFloor.setColor(Color.BLUE);
        mPathPaint_upperFloor.setAlpha(15);

        mPathPaint_FloorBelow = new Paint(Paint.ANTI_ALIAS_FLAG);
        mPathPaint_FloorBelow.setStyle(Paint.Style.STROKE);
        mPathPaint_FloorBelow.setStrokeWidth(1);
        mPathPaint_FloorBelow.setColor(Color.BLUE);
        mPathPaint_FloorBelow.setAlpha(18);

        mDirectPathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mDirectPathPaint.setStyle(Paint.Style.STROKE);
        mDirectPathPaint.setStrokeWidth(2);
        mDirectPathPaint.setColor(Color.GREEN);
        mDirectPathPaint.setPathEffect(new DashPathEffect(new float[]{5, 10, 15, 20}, 0));
        mDirectPathPaint.setAlpha(80);

        mCamera = new Matrix();
        mCameraInverse = new Matrix();

        // Register for surface callback events.
        getHolder().addCallback(this);

        //load plans
        if (FloorPlanNavigator.startFloorNum == 0)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage0);
        else if (FloorPlanNavigator.startFloorNum == 1)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage1);
        else if (FloorPlanNavigator.startFloorNum == 2)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage2);
        else if (FloorPlanNavigator.startFloorNum == 3)
            floorplan_image = BitmapFactory.decodeResource(getResources(), R.drawable.etage3);

    }


    //draw camera representation
    private void createUserMarkerShape(double pitch) {

        mUserMarkerPaint = new Paint();
        mUserMarkerPaint.setColor(Color.DKGRAY);
        mUserMarkerPaint.setStyle(Paint.Style.FILL);
        mUserMarkerPath = new Path();

        if (pitch == 0) {
            mUserMarkerPath.lineTo(-0.2f * SCALE, 0);
            mUserMarkerPath.lineTo(-0.2f * SCALE, -0.05f * SCALE);
            mUserMarkerPath.lineTo(0.2f * SCALE, -0.05f * SCALE);
            mUserMarkerPath.lineTo(0.2f * SCALE, 0);
            mUserMarkerPath.lineTo(0, 0);
            mUserMarkerPath.lineTo(0, -0.05f * SCALE);
            mUserMarkerPath.lineTo(-0.2f * SCALE, -0.5f * SCALE);
            mUserMarkerPath.lineTo(0.2f * SCALE, -0.5f * SCALE);
            mUserMarkerPath.lineTo(0, 0);
        } else {
            float factor = (float) Math.abs((Math.toDegrees(pitch + 90) % 180) / 100); //simulation of vertical direction of camera in 2D

            if (factor < 0.4f)
                factor = 0.4f;
            else if (factor > 1.1f)
                factor = 1.1f;

            mUserMarkerPath.lineTo(-0.2f * SCALE, 0);
            mUserMarkerPath.lineTo(-0.2f * SCALE, -0.05f * SCALE * factor);
            mUserMarkerPath.lineTo(0.2f * SCALE, -0.05f * SCALE * factor);
            mUserMarkerPath.lineTo(0.2f * SCALE, 0);
            mUserMarkerPath.lineTo(0, 0);
            mUserMarkerPath.lineTo(0, -0.05f * SCALE);
            mUserMarkerPath.lineTo(-0.2f * SCALE, -0.5f * SCALE * factor);
            mUserMarkerPath.lineTo(0.2f * SCALE, -0.5f * SCALE * factor);
            mUserMarkerPath.lineTo(0, 0);
        }

    }


    //draw upwards / downwards mark:
    void make_triangle(float[] start_point, int alpha, float scale_factor, boolean upwards) {

        //Triangle color
        mTopTrianglePaint = new Paint();
        mTopTrianglePaint.setColor(Color.parseColor("#009933")); //purple
        mTopTrianglePaint.setStyle(Paint.Style.FILL);
        mTopTrianglePaint.setAlpha(alpha);

        float my_scale = SCALE / scale_factor;

        if (upwards) {

            //Triangle toward top
            mTopTrianglePath = new Path();
            mTopTrianglePath.moveTo(0 + start_point[0], 0 + start_point[1]);
            mTopTrianglePath.lineTo(-0.2f * my_scale + start_point[0], -0.22f * my_scale + start_point[1]);
            mTopTrianglePath.lineTo(0.2f * my_scale + start_point[0], -0.22f * my_scale + start_point[1]);
            mTopTrianglePath.lineTo(0 + start_point[0], 0 + start_point[1]);
        } else {
            //Triangle toward top
            mTopTrianglePath = new Path();
            mTopTrianglePath.moveTo(0 + start_point[0], 0 + start_point[1]);
            mTopTrianglePath.lineTo(-0.2f * my_scale + start_point[0], 0.22f * my_scale + start_point[1]);
            mTopTrianglePath.lineTo(0.2f * my_scale + start_point[0], 0.22f * my_scale + start_point[1]);
            mTopTrianglePath.lineTo(0 + start_point[0], 0 + start_point[1]);
        }
    }


    @Override
    public void surfaceCreated(SurfaceHolder surfaceHolder) {

        mSurfaceHolder = surfaceHolder;
        mDrawThread = new RenderThread();
        mDrawThread.start();
        mIsDrawing = true;
    }

    @Override
    public void surfaceChanged(SurfaceHolder surfaceHolder, int i, int i1, int i2) {
        mSurfaceHolder = surfaceHolder;
    }


    @Override
    public void surfaceDestroyed(SurfaceHolder surfaceHolder) {
        mIsDrawing = false;
    }

    private void doDraw(Canvas canvas) {

        try {

            make_doDraw(canvas);
        } catch (Exception ex) {
            ex.printStackTrace();
        }

        // Draw a user / device marker.
        createUserMarkerShape(pitch);
        canvas.concat(mCameraInverse);
        canvas.drawPath(mUserMarkerPath, mUserMarkerPaint);

    }


    private void make_doDraw(Canvas canvas) {

        //keeping a shallow backup of coords for one run if basic info. is restored:
        List<Pose_Structure> temp_pose_keeper = FloorPlanNavigator.pose_keeper;

        List<FloorSwitch_Pose_Structure> temp_FloorSwitch_pose_keeper = FloorPlanNavigator.floorSwitch_pose_keeper;

        Paint circlePaint = new Paint();

        // Notify the activity so that it can use Tango to query the current device pose.
        if (mCallback != null) {
            mCallback.onPreDrawing();
        }

        // Erase the previous canvas image.
        canvas.drawColor(Color.WHITE);

        // Start drawing from the center of the canvas.
        translationX = canvas.getWidth() / 2f;
        translationY = canvas.getHeight() / 2f;
        translationX_map = canvas.getWidth() / 2f;
        translationY_map = canvas.getHeight() / 2f;

        canvas.translate(translationX, translationY); //move to canvas centre


        if (added_points.size() == 2) {
            FloorPlanNavigator.reset_seekBar(); //reset global scale
        } else if (last_scale != 2.5f && !(FloorPlanNavigator.user_allows_Scale_test && added_points.size() == 3))
            canvas.scale(last_scale, last_scale); //global scale


        //get Info. from inverse camera:
        float[] trans_vec = new float[9];
        mCameraInverse.getValues(trans_vec);


        //Update position and orientation based on the device position and orientation.
        canvas.concat(mCamera);

        //Calibration: Calculating matrix transformation:
        {
            canvas.save(Canvas.MATRIX_SAVE_FLAG);

            if (!FloorManager.camera_corner_pos_initialized && !FloorPlanNavigator.user_allows_Scale_test)
                canvas.translate(-dx, -dy); //to correct point adding!

            //cont. final calibration
            if (init_pose_fix != null) {
                canvas.translate(init_pose_fix[0], init_pose_fix[1]);

                if(init_angle_fix != Float.MAX_VALUE)
                    canvas.rotate( - init_angle_fix);
            }

            if (FloorManager.camera_corner_pos_initialized)
                canvas.concat(matrix_calibrated);

            canvas.translate(-translationX, -translationY); //init translation to meet centre

            if (FloorPlanNavigator.user_allows_Map) {

                Matrix checkPoint_matrix = canvas.getMatrix();
                canvas.translate(+translationX, +translationY);
                if (FloorManager.camera_corner_pos_initialized && added_points.size() == 3 && p2_static[0] > 0) {
                    canvas.scale(last_test_scale, last_test_scale , p2_static[0], p2_static[1]);
                    Log.i("Scale", ": " + last_test_scale);
                }
                canvas.translate(-translationX, -translationY);
                {

                    canvas.drawBitmap(floorplan_image, new Matrix(), null);

                }
                canvas.setMatrix(checkPoint_matrix);

            }

            canvas.translate(translationX, translationY);


            if (added_points != null && added_points.size() > 0) {

                final float fixed_distance_from_corner = 1f;
                mFurniturePaint.setColor(Color.RED);


                //Image Processing result: presentation (draw IP result)
                if (resized_IP_Bitmap != null && stored_old_val_of_head_corner != null && !FloorManager.camera_corner_pos_initialized) {

                    Paint alphaPaint = new Paint();
                    alphaPaint.setAlpha(200);

                    canvas.drawBitmap(Corners_detected_Bitmap, stored_old_val_of_head_corner[0] - (IP_patch_size / 2), stored_old_val_of_head_corner[1] - (IP_patch_size / 2), alphaPaint); // +5 is only for testing to see result!
                    //canvas.drawCircle(stored_old_val_of_head_corner[0] - (IP_patch_size / 2) + (float) corner_point_after_IP.x, stored_old_val_of_head_corner[1] - (IP_patch_size / 2) + (float) corner_point_after_IP.y, 1.2f, mFurniturePaint); //test of the head point applied on curr matrix

                }


                //Map Calibration phase:...................................................................................................
                if (resized_IP_Bitmap == null) {

                    triangular_ctrl_points_container_path = new Path();

                    if (added_points.size() != 3) {
                        for (int i = 0; i < added_points.size(); i++) {
                            if (i == 0)
                                triangular_ctrl_points_container_path.moveTo(added_points.get(i)[0], added_points.get(i)[1]);
                            else
                                triangular_ctrl_points_container_path.lineTo(added_points.get(i)[0], added_points.get(i)[1]);

                            int tmp_clr = mWallPaint.getColor();
                            mWallPaint.setColor(Color.RED);
                            canvas.drawCircle(added_points.get(i)[0], added_points.get(i)[1], SCALE / 35, mWallPaint);
                            mWallPaint.setColor(tmp_clr);
                        }
                    } else {


                        if (stored_old_val_of_head_corner == null) {
                            stored_old_val_of_head_corner = new float[2];
                            stored_old_val_of_head_corner[0] = added_points.get(1)[0];
                            stored_old_val_of_head_corner[1] = added_points.get(1)[1];

                            the_manuell_added_virtual_right_point = new float[2];
                            the_manuell_added_virtual_right_point[0] = added_points.get(2)[0];
                            the_manuell_added_virtual_right_point[1] = added_points.get(2)[1];

                        }


                        //Correct main corner position according to corner detection algorithm
                        if (resized_IP_Bitmap == null) {

                             p2_static[0] = added_points.get(1)[0]; //triangle's corner length is 0.0 (m): base
                            p2_static[1] = added_points.get(1)[1]; //triangle's corner length is 0.0 (m): base

                            //  canvas.concat(mCameraInverse);

                            int IP_patch_size = 150;

                            if (resized_IP_Bitmap == null) {
                                resized_IP_Bitmap = Bitmap.createBitmap(floorplan_image, (int) Math.abs((p2_static[0] - (IP_patch_size / 2)) + translationX), (int) Math.abs((p2_static[1] - (IP_patch_size / 2)) + translationY), IP_patch_size, IP_patch_size);
                                Corners_detected_Bitmap = detect_corner(resized_IP_Bitmap);
                            }


                            if (corner_point_left_after_IP != null && corner_point_right_after_IP != null && corner_point_after_IP != null) {

                                added_points.get(0)[0] = stored_old_val_of_head_corner[0] + (float) corner_point_left_after_IP.x - (IP_patch_size / 2);
                                added_points.get(0)[1] = stored_old_val_of_head_corner[1] + (float) corner_point_left_after_IP.y - (IP_patch_size / 2);
                                added_points.get(2)[0] = stored_old_val_of_head_corner[0] + (float) corner_point_right_after_IP.x - (IP_patch_size / 2);
                                added_points.get(2)[1] = stored_old_val_of_head_corner[1] + (float) corner_point_right_after_IP.y - (IP_patch_size / 2);
                                added_points.get(1)[0] += (float) corner_point_after_IP.x - (IP_patch_size / 2);
                                added_points.get(1)[1] += (float) corner_point_after_IP.y - (IP_patch_size / 2);

                                float[] point1_interpolated = new float[2];
                                float[] point3_interpolated = new float[2];

                                //correct positions: (interpolation)
                                point1_interpolated = FloorManager.point_on_line_with_fixed_length_from_first_point(fixed_distance_from_corner * SCALE, added_points.get(1)[0], added_points.get(1)[1], added_points.get(0)[0], added_points.get(0)[1]); //triangle's edge length is approx. 0.5 (m)
                                point3_interpolated = FloorManager.point_on_line_with_fixed_length_from_first_point(fixed_distance_from_corner * SCALE, added_points.get(1)[0], added_points.get(1)[1], added_points.get(2)[0], added_points.get(2)[1]); //triangle's edge length is approx. 0.5 (m)

                                p2_static[0] = added_points.get(1)[0]; //head
                                p2_static[1] = added_points.get(1)[1];
                                p1_static[0] = point1_interpolated[0]; //left
                                p1_static[1] = point1_interpolated[1];
                                p3_static[0] = point3_interpolated[0]; //right
                                p3_static[1] = point3_interpolated[1];


                                //decide which cornerCase_byAddedVirtualPoints have the added points:
                                if((p2_static[0] == p3_static[0]) && p3_static[0] < p1_static[0])
                                {
                                    cornerCase_byAddedVirtualPoints = 1;
                                }
                                else if((p2_static[0] == p3_static[0]) && p3_static[0] > p1_static[0])
                                {
                                    cornerCase_byAddedVirtualPoints = 2;
                                }
                                else if((p2_static[0] == p1_static[0]) && (p1_static[0] > p3_static[0]))
                                {
                                    cornerCase_byAddedVirtualPoints = 4;
                                }
                                else //if( (p3_static[0] == p1_static[0]) && p2_static[0] > p3_static[0])
                                {
                                    cornerCase_byAddedVirtualPoints = 3;
                                }

                                FloorPlanNavigator.showsToastOnUiThread("direction_type= " + cornerCase_byAddedVirtualPoints, FloorPlanNavigator.UIThread_accessibility_providing_activity);


                            }

                        }


                        virtual_point_middle_of_p1_and_p3 = new float[2];


                        for (int i = 0; i < added_points.size(); i++) {
                            if (i == 2) // draw the corrected bzw. calculated 3rd point:
                                triangular_ctrl_points_container_path.lineTo(p3_static[0], p3_static[1]);
                            else if (i == 0)
                                triangular_ctrl_points_container_path.moveTo(p1_static[0], p1_static[1]);
                            else
                                triangular_ctrl_points_container_path.lineTo(p2_static[0], p2_static[1]);
                        }
                        //correct the middle point w.r.t the new perpendicular point
                        virtual_point_middle_of_p1_and_p3[0] = (p1_static[0] + p3_static[0]) / 2;
                        virtual_point_middle_of_p1_and_p3[1] = (p1_static[1] + p3_static[1]) / 2;

                        mFurniturePaint.setColor(Color.RED);


                    }
                }


                //Visualisation of the corrected ctrl points after calibration phase
                if (triangular_ctrl_points_container_path != null && !FloorManager.camera_corner_pos_initialized) {

                    mFurniturePaint.setColor(Color.YELLOW);
                    mFurniturePaint.setAlpha(40);
                    canvas.drawPath(triangular_ctrl_points_container_path, mFurniturePaint);
                    mFurniturePaint.setAlpha(100);
                    //triangular_ctrl_points_container_path.reset();
                    mFurniturePaint.setColor(Color.RED);

                    //to draw next points after final calibration phase (later)
                    canvas.drawCircle(p1_static[0], p1_static[1], 1f, mFurniturePaint);
                    canvas.drawCircle(p2_static[0], p2_static[1], 1f, mFurniturePaint);
                    canvas.drawCircle(p3_static[0], p3_static[1], 1f, mFurniturePaint);

                    mFurniturePaint.setColor(Color.BLUE);
                    canvas.drawCircle(virtual_point_middle_of_p1_and_p3[0], virtual_point_middle_of_p1_and_p3[1], .4f, mFurniturePaint);
                    mFurniturePaint.setColor(Color.RED);

                }


                // End real-virt. Calibration phase (After Map Calibration phase is executed):
                if (!FloorManager.camera_corner_pos_initialized && FloorManager.angle_between_3p_is_right && added_points.size() == 3 && p3_static[0] != 0.0f) { //????????????????? rotate when corner derives

                    //Translate to new origin:
                    //Source: http://www.inf.ed.ac.uk/teaching/courses/cg/lectures/cg3_2013.pdf
                    //Source: http://www.cs.brandeis.edu/~cs155/Lecture_06.pdf
                    //Source: http://web.cse.ohio-state.edu/~parent.1/classes/581/Lectures/5.2DtransformsAhandout.pdf


                    //calculate translation
                    float X_translate_virtual_2_real = (FloorManager.Pz[0] * SCALE) - p2_static[0];
                    float Y_translate_virtual_2_real = (-FloorManager.Pz[1] * SCALE) - p2_static[1];

                    //rescale current matrix to original scale before calibration
                    last_scale = 2.5f;


                    //3 points of the needed angle to rotate (between real and virtual corners):
                    float[] virtual_head_pt = {0 + p2_static[0], 0 + p2_static[1]};
                    float[] real_head_pt = {(FloorManager.Pz[0] * SCALE) - X_translate_virtual_2_real, (-FloorManager.Pz[1] * SCALE) - Y_translate_virtual_2_real};

                    float[] virtual_right_pt = {0 + p3_static[0], 0 + p3_static[1]};
                    float[] real_right_pt = {(FloorManager.Pr2[0] * SCALE) - X_translate_virtual_2_real, (-FloorManager.Pr2[1] * SCALE) - Y_translate_virtual_2_real};

                    float[] virtual_left_pt = {0 + p1_static[0], 0 + p1_static[1]};
                    float[] real_left_pt = {(FloorManager.Pr1[0] * SCALE) - X_translate_virtual_2_real, (-FloorManager.Pr1[1] * SCALE) - Y_translate_virtual_2_real};



                    //ReCalibrate the virtual points to represent the real ones (about lengths and triangle rules)
                    float length_of_left_real_edge = (float) FloorManager.distance(virtual_head_pt[0], virtual_head_pt[1], real_left_pt[0], real_left_pt[1]);
                    float[] virtual_left_pt_calibrated = FloorManager.point_on_line_with_fixed_length_from_first_point(length_of_left_real_edge , virtual_head_pt[0], virtual_head_pt[1], virtual_left_pt[0], virtual_left_pt[1]);

                    float length_of_right_real_edge = (float) FloorManager.distance(real_head_pt[0], real_head_pt[1], real_right_pt[0], real_right_pt[1]);
                    float[] virtual_right_pt_calibrated = FloorManager.point_on_line_with_fixed_length_from_first_point(length_of_right_real_edge , real_head_pt[0], real_head_pt[1], virtual_right_pt[0], virtual_right_pt[1]);

                    //find projection of virtual_head_pt on line virtual_right_pt, virtual_left_pt
                    virtual_projection_of_head_on_line_LeftRight_calibrated_pt = new float[2];
                    float k = (float) (((virtual_right_pt_calibrated[1] - virtual_left_pt_calibrated[1]) * (virtual_head_pt[0] - virtual_left_pt_calibrated[0]) - (virtual_right_pt_calibrated[0] - virtual_left_pt_calibrated[0]) * (virtual_head_pt[1] - virtual_left_pt_calibrated[1])) / (Math.pow((virtual_right_pt_calibrated[1] - virtual_left_pt_calibrated[1]), 2) + Math.pow((virtual_right_pt_calibrated[0] - virtual_left_pt_calibrated[0]), 2)));
                    virtual_projection_of_head_on_line_LeftRight_calibrated_pt[0] = virtual_head_pt[0] - k * (virtual_right_pt_calibrated[1] - virtual_left_pt_calibrated[1]);
                    virtual_projection_of_head_on_line_LeftRight_calibrated_pt[1] = virtual_head_pt[1] + k * (virtual_right_pt_calibrated[0] - virtual_left_pt_calibrated[0]);

                    //calculating the virtual point that represent the correct distance of virt. camera from the head
                    float length_of_displacement =  FloorManager.Pz[1] * SCALE;
                    float[] correct_distance_of_camera_from_head = FloorManager.point_on_line_with_fixed_length_from_first_point(length_of_displacement , virtual_head_pt[0], virtual_head_pt[1], virtual_projection_of_head_on_line_LeftRight_calibrated_pt[0], virtual_projection_of_head_on_line_LeftRight_calibrated_pt[1]);
                    temp_point_show = correct_distance_of_camera_from_head;

                    //Calculating the angle between head, left, virtual_projection_of_head_on_line_LeftRight_calibrated_pt
                    float angle_head_left_progectionOnLeftRight = FloorManager.angle_between_3_points_wrt_first_one(virtual_head_pt[0], virtual_head_pt[1], virtual_left_pt_calibrated[0], virtual_left_pt_calibrated[1], virtual_projection_of_head_on_line_LeftRight_calibrated_pt[0], virtual_projection_of_head_on_line_LeftRight_calibrated_pt[1]);

                    //map the service starting position to the calibrated one (translation)
                    float total_trans_x = (-translationX + dx) - correct_distance_of_camera_from_head[0];
                    float total_trans_y = (-translationY + dy) - correct_distance_of_camera_from_head[1];

                    //------------------------------------------------------

                    //keep values for a later visualisation
                    testing_after_calibrating_real_points = new float[6];
                    testing_after_calibrating_real_points[0] = real_head_pt[0];
                    testing_after_calibrating_real_points[1] = real_head_pt[1];
                    testing_after_calibrating_real_points[2] = real_right_pt[0];
                    testing_after_calibrating_real_points[3] = real_right_pt[1];
                    testing_after_calibrating_real_points[4] = real_left_pt[0];
                    testing_after_calibrating_real_points[5] = real_left_pt[1];

                    testing_after_calib_virt_points = new float[6];
                    testing_after_calib_virt_points[0] = virtual_head_pt[0];
                    testing_after_calib_virt_points[1] = virtual_head_pt[1];
                    testing_after_calib_virt_points[2] = virtual_right_pt_calibrated[0];
                    testing_after_calib_virt_points[3] = virtual_right_pt_calibrated[1];
                    testing_after_calib_virt_points[4] = virtual_left_pt_calibrated[0];
                    testing_after_calib_virt_points[5] = virtual_left_pt_calibrated[1];

                    //calculate angle between the left edges of both real and virtual corners
                    float angle_vrtl_real = FloorManager.angle_between_3_points_wrt_first_one(virtual_head_pt[0], virtual_head_pt[1], virtual_left_pt_calibrated[0], virtual_left_pt_calibrated[1], real_left_pt[0], real_left_pt[1]);
                    angle_btwn_vrtl_real_corners = angle_vrtl_real; //to represent the real points later (here no use)


                    //Main Calibration
                    //Translate: origin to virtual_projection_of_head_on_line_LeftRight_calibrated_pt
                    {
                        canvas.translate(total_trans_x, total_trans_y);
                    }

                    //Rotate (includes 4 corner's cases):
                    {
                         float clibration_angle = 0.0f;

                        if (cornerCase_byAddedVirtualPoints == 1)
                            clibration_angle = angle_head_left_progectionOnLeftRight + 90;
                        else  if (cornerCase_byAddedVirtualPoints == 2)
                            clibration_angle = angle_head_left_progectionOnLeftRight + 270;
                        else  if (cornerCase_byAddedVirtualPoints == 3)
                            clibration_angle = angle_head_left_progectionOnLeftRight;
                        else  if (cornerCase_byAddedVirtualPoints == 4)
                            clibration_angle = angle_head_left_progectionOnLeftRight + 180;

                        canvas.rotate(clibration_angle, correct_distance_of_camera_from_head[0], correct_distance_of_camera_from_head[1]);

                    }

                   /* //old methods - not valid (thinking way):
                    //Corrections set 2 //Now included with the last calc. "clibration_angle"
                    {
                        canvas.translate(FloorManager.Pz[0]* SCALE, 0);

                        float angle_corrected_2 = FloorManager.angle_between_3_points_wrt_first_one( FloorManager.Pz[0]* SCALE, 0, virtual_head_pt[0], virtual_head_pt[1], FloorManager.current_corresponding_point[0]* SCALE, FloorManager.current_corresponding_point[1]* SCALE);

                    }

                    //Scale: //NOT POSSIBLE --> Manually calibrated by user
                    {
                        //calculate distance between real pt and virt pt:
                        //float dist_btwn_2_ends = (float) FloorManager.distance(right_virt_pt[0], right_virt_pt[1], real_left_pt[0], real_left_pt[1]);
                        // canvas.scale(dist_btwn_2_ends / (fixed_distance_from_corner * SCALE), dist_btwn_2_ends / (fixed_distance_from_corner * SCALE), p2_static[0], p2_static[1]);
                        //canvas.scale((FloorManager.Pz[1] * SCALE), (FloorManager.Pz[1] * SCALE), virtual_head_pt[0], virtual_head_pt[1]); //scale to Z value
                    }


                    //Floor Height: store the last floor height (supposed as a valid shot where walls corner + ceiling + ground are shown): NOT VALID!
                    {

                        if (!FloorPlanNavigator.different_floors_heights && FloorManager.highest_y != Float.MAX_VALUE && FloorManager.highest_y != Float.MIN_VALUE && !FloorPlanNavigator.height_manually_adjusted) {
                            FloorPlanNavigator.floorHeight = Math.abs(FloorManager.highest_y - FloorManager.lowest_y);
                            FloorPlanNavigator.showsToastOnUiThread("Updated Floor Height to " + FloorPlanNavigator.floorHeight, FloorPlanNavigator.UIThread_accessibility_providing_activity);
                        } else if (FloorPlanNavigator.height_manually_adjusted)
                            FloorPlanNavigator.showsToastOnUiThread("Not updating height because it is already manually adjusted in this session!" + FloorPlanNavigator.floorHeight, FloorPlanNavigator.UIThread_accessibility_providing_activity);

                    }
                    */

                    //Update scale factor to the old value from prev. session:
                    {
                            last_test_scale = FloorPlanNavigator.init_scale_factor; //the last selected scale factor from other session
                    }


                    matrix_calibrated = canvas.getMatrix(); //the result calibrated matrix
                    FloorManager.camera_corner_pos_initialized = true; // calibration phase completed
                    FloorPlanNavigator.showsToastOnUiThread("Calibrated!", FloorPlanNavigator.UIThread_accessibility_providing_activity);

                    FloorPlanNavigator.reset_seekBar(); // reset scaling

                    //from now on consider poses to affect the floor plan
                    allow_update_cam_poses = true;
                }


            }

            //Representation
            if (FloorManager.camera_corner_pos_initialized && testing_after_calibrating_real_points != null && matrix_calibrated != null) {

                int col = circlePaint.getColor();

                if (cornerCase_byAddedVirtualPoints != 4) //only for representation..
                {
                    Matrix checkPoint_matrix = canvas.getMatrix();
                    canvas.rotate(angle_btwn_vrtl_real_corners, testing_after_calibrating_real_points[0], testing_after_calibrating_real_points[1]); //only presentation, no functionality

                    circlePaint.setColor(Color.CYAN); //real
                    circlePaint.setAlpha(200);
                    canvas.drawCircle(testing_after_calibrating_real_points[0], testing_after_calibrating_real_points[1], 6f, circlePaint);
                    canvas.drawCircle(testing_after_calibrating_real_points[2], testing_after_calibrating_real_points[3], 6f, circlePaint);
                    canvas.drawCircle(testing_after_calibrating_real_points[4], testing_after_calibrating_real_points[5], 6f, circlePaint);

                    canvas.setMatrix(checkPoint_matrix);
                }

                //virtual
                //Visualisation of the corrected ctrl points after calibration phase

                //connect virt. points:
                triangular_ctrl_points_container_path = new Path();
                triangular_ctrl_points_container_path.moveTo(testing_after_calib_virt_points[0], testing_after_calib_virt_points[1]);
                triangular_ctrl_points_container_path.lineTo(testing_after_calib_virt_points[2], testing_after_calib_virt_points[3]);
                triangular_ctrl_points_container_path.lineTo(testing_after_calib_virt_points[4], testing_after_calib_virt_points[5]);

                mFurniturePaint.setColor(Color.YELLOW);
                mFurniturePaint.setAlpha(40);
                canvas.drawPath(triangular_ctrl_points_container_path, mFurniturePaint);
                mFurniturePaint.setAlpha(100);
                //triangular_ctrl_points_container_path.reset();
                mFurniturePaint.setColor(Color.RED);

                circlePaint.setAlpha(120);
                canvas.drawCircle(testing_after_calib_virt_points[0], testing_after_calib_virt_points[1], 4f, mFurniturePaint);
                canvas.drawCircle(testing_after_calib_virt_points[2], testing_after_calib_virt_points[3], 3f, mFurniturePaint);
                canvas.drawCircle(testing_after_calib_virt_points[4], testing_after_calib_virt_points[5], 2f, mFurniturePaint);

                mFurniturePaint.setColor(Color.BLUE);
                canvas.drawCircle(virtual_projection_of_head_on_line_LeftRight_calibrated_pt[0], virtual_projection_of_head_on_line_LeftRight_calibrated_pt[1], 1f, mFurniturePaint);
                mFurniturePaint.setColor(Color.RED);

                circlePaint.setColor(col); //reset col
            }


            canvas.restore();

        }




// Position Rotation & Depth inputs operations: ............................................................................................................................

        canvas.save(Canvas.MATRIX_SAVE_FLAG);
        canvas.translate(-1 * translationX, -1 * translationY);

        make_grid(canvas);
        canvas.restore();

        //store the initial orientation points of canvas
        if(init_orientation_path == null) {

            virtual_start_direction_line = new float[4]; //2 points: start, end
            virtual_start_direction_line[0] = 0;
            virtual_start_direction_line[1] = 0;
            virtual_start_direction_line[2] = 0;
            virtual_start_direction_line[3] = -2;

            init_orientation_path = new Path();
            init_orientation_path.moveTo(virtual_start_direction_line[0] * SCALE, virtual_start_direction_line[1] * SCALE);
            init_orientation_path.lineTo(virtual_start_direction_line[2] * SCALE, virtual_start_direction_line[3] * SCALE);
        }

        Paint tmp_pt = new Paint(Color.RED);
        tmp_pt.setStrokeWidth(1);
        tmp_pt.setAlpha(100);
        canvas.drawLine(virtual_start_direction_line[0] * SCALE, virtual_start_direction_line[1] * SCALE, virtual_start_direction_line[2] * SCALE, virtual_start_direction_line[3] * SCALE, tmp_pt);


        // Draw all the polygons. Make a shallow copy in case mPolygons is reset while rendering.
        List<TangoPolygon> drawPolygons = mPolygons;

        // 2D calculation of scale from depth-based camera matrix
        float scalex = trans_vec[Matrix.MSCALE_X];
        float skewy = trans_vec[Matrix.MSKEW_Y];
        float rScale = (float) Math.sqrt(scalex * scalex + skewy * skewy);

        // and calculate rotation: degree
        float rAngle = Math.round(Math.atan2(trans_vec[Matrix.MSKEW_X], trans_vec[Matrix.MSCALE_X]) * (180 / Math.PI));

        cam_rot = rAngle;

        //Store last depth-based pose:
        Pose_Structure tf = new Pose_Structure(trans_vec[Matrix.MTRANS_X], trans_vec[Matrix.MTRANS_Y]);
        tf.setRotation(rAngle);
        tf.setFloor_number(FloorPlanNavigator.currFloorNum);
        temp_pose_keeper.add(tf);
        FloorPlanNavigator.pose_keeper.add(tf);


        //cont. final calibration
        if(FloorManager.camera_corner_pos_initialized && init_pose_fix == null && temp_pose_keeper.get(temp_pose_keeper.size() - 1).getTrans_X() != 0 && temp_pose_keeper.get(temp_pose_keeper.size() - 1).getTrans_Y() != 0) {
            init_pose_fix = new float[2];
            init_pose_fix[0] = temp_pose_keeper.get(temp_pose_keeper.size() - 1).getTrans_X();
            init_pose_fix[1]= temp_pose_keeper.get(temp_pose_keeper.size() - 1).getTrans_Y();

            init_angle_fix = rAngle;
        }


        //Draw a starting position circle
        circlePaint.setColor(Color.RED);
        circlePaint.setAntiAlias(true);
        circlePaint.setAlpha(70);
        if (temp_pose_keeper.size() != 0)
            canvas.drawCircle(temp_pose_keeper.get(0).getTrans_X(), temp_pose_keeper.get(0).getTrans_Y(), SCALE / 2, circlePaint);


        Paint bg_paint = new Paint(); //init color pink
        circlePaint.setAlpha(70);
        bg_paint.setColor(Color.parseColor("#FF0080"));
        Paint paint = bg_paint;
        int points_to_read = 100;
        float[] p_wall_position = {0, 0};
        float[] p_furniture_position = {0, 0};
        float[] p_space_position = {0, 0};
        float[] occlusion_pos = {0, 0};
        float presentation_size_of_detected_objects = 1.0f;


        float distance_to_furthest_point = Float.MIN_VALUE;
        float[] furthest_point = {0, 0};

        for (int i = 0; i < drawPolygons.size(); i++) {

            if (drawPolygons.size() > 0) {
                TangoPolygon polygon = drawPolygons.get(i);

                if (polygon.vertices2d.size() > 1) {

                    for (int s = 0; s < polygon.vertices2d.size(); s++) {

                        switch (polygon.layer) {
                            case TangoPolygon.TANGO_3DR_LAYER_FURNITURE:
                                paint = mFurniturePaint;
                                break;
                            case TangoPolygon.TANGO_3DR_LAYER_SPACE:
                                paint = mSpacePaint;
                                break;
                            case TangoPolygon.TANGO_3DR_LAYER_WALLS:
                                paint = mWallPaint;
                                break;
                            default:
                                paint = bg_paint;
                                Log.w(TAG, "Ignoring polygon with unknown layer value: " + polygon.layer);
                                continue;
                        }


                        if (paint != null) {

                            double dist_btwn_pointer_and_Tango_obj_point = FloorManager.distance(polygon.vertices2d.get(s)[0], polygon.vertices2d.get(s)[1], 0, 0); // Does not get corners (only)
                            if (distance_to_furthest_point < dist_btwn_pointer_and_Tango_obj_point) {

                                distance_to_furthest_point = (float) dist_btwn_pointer_and_Tango_obj_point;
                                furthest_point[0] = polygon.vertices2d.get(s)[0];
                                furthest_point[1] = polygon.vertices2d.get(s)[1];

                            }

                        }
                    }

                }
            }
        }

        if(furthest_point[0] > 1 && furthest_point[1] > 1) {
            circlePaint.setColor(Color.GREEN);
            circlePaint.setAlpha(60);
            canvas.drawCircle(furthest_point[0] * SCALE, furthest_point[1] * SCALE, presentation_size_of_detected_objects * 6, circlePaint);
        }


        mPolygons.clear();
        drawPolygons.clear();


        final float presentation_size_of_real_points = 5.0f;
        paint = mWallPaint;

        //draw representative Occlusion: only representation, does not effect calcs
        if (avg_objects_depth <= FloorPlanNavigator.limit_far && avg_objects_depth >= FloorPlanNavigator.limit_near) { //show only the near object (occlusion) : less than ca. one a half meter //distance(tf.getTrans_X(), tf.getTrans_Y(), p[0], p[1]) <= 1.09

            canvas.concat(mCameraInverse); //to get the virt. world view

            canvas.drawCircle(0 * SCALE, -avg_objects_depth * SCALE, presentation_size_of_real_points / 2, paint);


            if (FloorManager.angle_between_3p_is_right) {

                last_calculated_real_points[0] = FloorManager.Pr1[0];
                last_calculated_real_points[1] = FloorManager.Pr1[1];
                last_calculated_real_points[2] = FloorManager.Pz[0];
                last_calculated_real_points[3] = FloorManager.Pz[1];
                last_calculated_real_points[4] = FloorManager.Pr2[0];
                last_calculated_real_points[5] = FloorManager.Pr2[1];

                paint.setAlpha(90);
                canvas.drawCircle(FloorManager.Pz[0] * SCALE, -FloorManager.Pz[1] * SCALE, presentation_size_of_real_points, paint);
                paint.setAlpha(80);
                canvas.drawCircle(FloorManager.Pr1[0] * SCALE, -FloorManager.Pr1[1] * SCALE, presentation_size_of_real_points, paint);
                paint.setAlpha(70);
                canvas.drawCircle(FloorManager.Pr2[0] * SCALE, -FloorManager.Pr2[1] * SCALE, presentation_size_of_real_points, paint);
                paint.setAlpha(100);
            }

            canvas.concat(mCamera); // reset to real world


            if (avg_objects_depth <= FloorPlanNavigator.limit_near && FloorPlanNavigator.user_allows_vibration && FloorPlanNavigator.dev_allows_vibration_yet) //make a vibration by critical depth
            {
                Android_Functionality_Class.make_vibration(35, getContext());
                FloorPlanNavigator.dev_allows_vibration_yet = false;
            }

        }


        //Draw walking path for the current floor
        boolean init_point_mode = true;

        if (temp_pose_keeper.size() > 1)
            for (int i = 0; i < temp_pose_keeper.size(); i++) {
                if (temp_pose_keeper.size() != 0 && temp_pose_keeper.get(i) != null && temp_pose_keeper.get(i).getFloor_number() == FloorPlanNavigator.currFloorNum) {
                    float[] point = {temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y()};

                    if (init_point_mode) {
                        mBashPath.moveTo(point[0], point[1]);
                        init_point_mode = false;

                    } else {

                        if (FloorManager.distance(temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y(), temp_pose_keeper.get(i - 1).getTrans_X(), temp_pose_keeper.get(i - 1).getTrans_Y()) > 50) //if there is a jump (wrong measurements caused by AL): 50 CM
                        {
                            Paint err_paint = new Paint(Paint.ANTI_ALIAS_FLAG);
                            err_paint.setColor(Color.RED);
                            err_paint.setStyle(Paint.Style.STROKE);
                            err_paint.setStrokeWidth(4);
                            err_paint.setAlpha(90);
                            err_paint.setPathEffect(new DashPathEffect(new float[]{5, 10, 15, 20}, 0));

                            PointF p1 = new PointF(temp_pose_keeper.get(i - 1).getTrans_X(), temp_pose_keeper.get(i - 1).getTrans_Y());
                            PointF p2 = new PointF(temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y());

                            Path err_path = new Path();
                            err_path.moveTo(p1.x, p1.y);
                            err_path.lineTo(p2.x, p2.y);

                            canvas.drawPath(err_path, err_paint);
                        } else {

                            if (temp_pose_keeper.get(i).isFirst_point_of_path_in_this_floor())
                                mBashPath.moveTo(point[0], point[1]);

                            else if (temp_pose_keeper.get(i).isLast_point_of_path_in_this_floor()) {
                                if (!mBashPath.isEmpty())
                                    canvas.drawPath(mBashPath, mPathPaint_currentFloor);
                                mBashPath.reset();
                            } else
                                mBashPath.lineTo(point[0], point[1]);
                        }
                    }

                }
            }

        if (!mBashPath.isEmpty())
            canvas.drawPath(mBashPath, mPathPaint_currentFloor);
        mBashPath.reset();

        init_point_mode = true;
        //Draw walking path for the upper floor (if any)
        if (temp_pose_keeper.size() != 0)
            for (int i = 0; i < temp_pose_keeper.size(); i++) {
                if (temp_pose_keeper.get(i).getFloor_number() == FloorPlanNavigator.currFloorNum + 1) {
                    float[] point = {temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y()};

                    if (init_point_mode) {
                        mBashPath.moveTo(point[0], point[1]);
                        init_point_mode = false;
                    } else {
                        if (FloorManager.distance(temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y(), temp_pose_keeper.get(i - 1).getTrans_X(), temp_pose_keeper.get(i - 1).getTrans_Y()) > 50) //if there is a jump (wrong measurements caused by AL): 50 CM
                        {
                            Paint err_paint = new Paint(Paint.ANTI_ALIAS_FLAG);
                            err_paint.setColor(Color.RED);
                            err_paint.setStyle(Paint.Style.STROKE);
                            err_paint.setStrokeWidth(4);
                            err_paint.setAlpha(70);
                            err_paint.setPathEffect(new DashPathEffect(new float[]{5, 10, 15, 20}, 0));

                            PointF p1 = new PointF(temp_pose_keeper.get(i - 1).getTrans_X(), temp_pose_keeper.get(i - 1).getTrans_Y());
                            PointF p2 = new PointF(temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y());

                            Path err_path = new Path();
                            err_path.moveTo(p1.x, p1.y);
                            err_path.lineTo(p2.x, p2.y);

                            canvas.drawPath(err_path, err_paint);
                        } else {
                            if (temp_pose_keeper.get(i).isFirst_point_of_path_in_this_floor())
                                mBashPath.moveTo(point[0], point[1]);
                            else if (temp_pose_keeper.get(i).isLast_point_of_path_in_this_floor()) {
                                if (!mBashPath.isEmpty())
                                    canvas.drawPath(mBashPath, mPathPaint_upperFloor);
                                mBashPath.reset();
                            } else
                                mBashPath.lineTo(point[0], point[1]);
                        }
                    }

                }
            }

        if (!mBashPath.isEmpty())
            canvas.drawPath(mBashPath, mPathPaint_upperFloor);
        mBashPath.reset();

        init_point_mode = true;
        //Draw walking path for the floor below (if any)
        if (temp_pose_keeper.size() != 0)
            for (int i = 0; i < temp_pose_keeper.size(); i++) {
                if (temp_pose_keeper.get(i).getFloor_number() == FloorPlanNavigator.currFloorNum - 1) {
                    float[] point = {temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y()};

                    if (init_point_mode) {
                        mBashPath.moveTo(point[0], point[1]);
                        init_point_mode = false;
                    } else {
                        if (FloorManager.distance(temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y(), temp_pose_keeper.get(i - 1).getTrans_X(), temp_pose_keeper.get(i - 1).getTrans_Y()) > 50) //if there is a jump (wrong measurements caused by AL): 50 CM
                        {
                            Paint err_paint = new Paint(Paint.ANTI_ALIAS_FLAG);
                            err_paint.setColor(Color.RED);
                            err_paint.setStyle(Paint.Style.STROKE);
                            err_paint.setStrokeWidth(4);
                            err_paint.setAlpha(60);
                            err_paint.setPathEffect(new DashPathEffect(new float[]{5, 10, 15, 20}, 0));

                            PointF p1 = new PointF(temp_pose_keeper.get(i - 1).getTrans_X(), temp_pose_keeper.get(i - 1).getTrans_Y());
                            PointF p2 = new PointF(temp_pose_keeper.get(i).getTrans_X(), temp_pose_keeper.get(i).getTrans_Y());

                            Path err_path = new Path();
                            err_path.moveTo(p1.x, p1.y);
                            err_path.lineTo(p2.x, p2.y);

                            canvas.drawPath(err_path, err_paint);
                        } else {
                            if (temp_pose_keeper.get(i).isFirst_point_of_path_in_this_floor())
                                mBashPath.moveTo(point[0], point[1]);

                            else if (temp_pose_keeper.get(i).isLast_point_of_path_in_this_floor()) {
                                if (!mBashPath.isEmpty())
                                    canvas.drawPath(mBashPath, mPathPaint_FloorBelow);
                                mBashPath.reset();
                            } else
                                mBashPath.lineTo(point[0], point[1]);
                        }
                    }

                }
            }

        if (!mBashPath.isEmpty())
            canvas.drawPath(mBashPath, mPathPaint_FloorBelow);
        mBashPath.reset();


        //Check when we just switched floor
        if (mIsFloorSwitchingMode) {
            if (temp_pose_keeper.size() > 2) {
                temp_pose_keeper.get(temp_pose_keeper.size() - 2).setLast_point_of_path_in_this_floor(true); //last point in the last floor
                temp_pose_keeper.get(temp_pose_keeper.size() - 1).setFirst_point_of_path_in_this_floor(true);//first point in the current floor
                FloorSwitch_Pose_Structure fps = new FloorSwitch_Pose_Structure(temp_pose_keeper.get(temp_pose_keeper.size() - 1),
                        temp_pose_keeper.get(temp_pose_keeper.size() - 2).getPose_id(),
                        temp_pose_keeper.get(temp_pose_keeper.size() - 2).getFloor_number(),
                        mFloorSwitchType);
                temp_FloorSwitch_pose_keeper.add(fps);
                FloorPlanNavigator.floorSwitch_pose_keeper.add(fps);
                mIsFloorSwitchingMode = false;
            }
        }

        //mark floor switch position with a green oriented marker
        if (temp_FloorSwitch_pose_keeper.size() > 0) {
            float scale_factor = 1;
            int alpha = 80;
            boolean upwards = true;

            for (FloorSwitch_Pose_Structure ps : temp_FloorSwitch_pose_keeper) { //ps: the pose of the floor-switch by only the new floor (first point of new floor)

                boolean allow_draw_marker = false;

                if (ps.previous_Floor_number == FloorPlanNavigator.currFloorNum - 1 && ps.floorSwitchType == FloorSwitch.switch_upwards) { //prev floor is below
                    upwards = false;
                    allow_draw_marker = true;
                } else if (ps.previous_Floor_number == FloorPlanNavigator.currFloorNum + 1 && ps.floorSwitchType == FloorSwitch.switch_downwards) { //prev floor is upper
                    upwards = true;
                    allow_draw_marker = true;
                } else if (ps.previous_Floor_number == FloorPlanNavigator.currFloorNum && ps.floorSwitchType == FloorSwitch.switch_downwards) { //prev floor is upper
                    upwards = true;
                    allow_draw_marker = true;
                } else if (ps.previous_Floor_number == FloorPlanNavigator.currFloorNum && ps.floorSwitchType == FloorSwitch.switch_upwards) { //prev floor is upper
                    upwards = false;
                    allow_draw_marker = true;
                }

                if (allow_draw_marker) {
                    canvas.save(Canvas.MATRIX_SAVE_FLAG);
                    canvas.rotate(-rAngle, ps.getPose_structure().getTrans_X(), ps.getPose_structure().getTrans_Y());
                    make_triangle(new float[]{ps.getPose_structure().getTrans_X(), ps.getPose_structure().getTrans_Y()}, alpha, scale_factor, upwards);
                    canvas.drawPath(mTopTrianglePath, mTopTrianglePaint);
                    canvas.restore();
                }

            }
        }


        //connect current point to the start point
        if (temp_pose_keeper.size() != 0) {

            mDirectPath.moveTo(temp_pose_keeper.get(0).getTrans_X(), temp_pose_keeper.get(0).getTrans_Y());
            float[] end_point = {temp_pose_keeper.get(temp_pose_keeper.size() - 1).getTrans_X(), temp_pose_keeper.get(temp_pose_keeper.size() - 1).getTrans_Y()};
            mDirectPath.lineTo(end_point[0], end_point[1]);
            canvas.drawPath(mDirectPath, mDirectPathPaint);
            mDirectPath.reset();

        }


    }


    //to get OpenCV Image color format: https://stackoverflow.com/questions/36394368/converting-bitmap-to-mat-in-java-for-image-processing
    private Bitmap JPGtoRGB888(Bitmap img) {
        Bitmap result = null;

        int numPixels = img.getWidth() * img.getHeight();
        int[] pixels = new int[numPixels];

        //get jpeg pixels, each int is the color value of one pixel
        img.getPixels(pixels, 0, img.getWidth(), 0, 0, img.getWidth(), img.getHeight());

        //create bitmap in appropriate format
        result = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.ARGB_8888);

        //Set RGB pixels
        result.setPixels(pixels, 0, result.getWidth(), 0, 0, result.getWidth(), result.getHeight());

        return result;
    }



    //http://docs.opencv.org/2.4/doc/tutorials/features2d/trackingmotion/harris_detector/harris_detector.html
    //https://stackoverflow.com/questions/41413509/opencv-java-harris-corner-detection
    private Bitmap detectNearestCorner_Harris(Mat src, int thresh) {

        Mat src_gray = new Mat();
        Mat src_binary = new Mat(); //does not calculate threshold, it's taken as para.
        Mat dst = new Mat();
        Mat dst_norm = new Mat();
        Mat dst_norm_scaled = new Mat();


        // Detector parameters
        int blockSize = 6;
        int apertureSize = 5;
        double k = 0.01;

        // Detecting corner
        Imgproc.cvtColor(src, src_gray, Imgproc.COLOR_BGR2GRAY);
        //Imgproc.threshold(src_gray, src_binary, 2, 255, Imgproc.THRESH_TRUNC);
        Imgproc.cornerHarris(src_gray, dst, blockSize, apertureSize, k);

        // Normalizing
        Core.normalize(dst, dst_norm, 0, 255, Core.NORM_MINMAX);
        Core.convertScaleAbs(dst_norm, dst_norm_scaled);

        // Finding detected corners
        List<org.opencv.core.Point> ALLcorners = new ArrayList<org.opencv.core.Point>(); //to keep all detected points
        for (int j = 0; j < dst_norm.rows(); j++) {
            for (int i = 0; i < dst_norm.cols(); i++) {
                if (dst_norm.get(j, i)[0] > thresh) {
                    // Imgproc.circle(dst_norm_scaled, new org.opencv.core.Point(i, j), 2, new Scalar(50), 1);
                    ALLcorners.add(new org.opencv.core.Point(i, j));
                }
            }
        }

        //calculating the nearest detected corner to the head angle selected by the user
        org.opencv.core.Point center_point = new org.opencv.core.Point(dst_norm.cols() / 2, dst_norm.rows() / 2);
        double distance_to_center = Float.MAX_VALUE;
        int y_direction = 1; //from center of patch go to (-) left, or (+) right to reach the detected point
        int x_direction = 1;
        int the_nearest_point_index = -1;

        for (int i = 0; i < ALLcorners.size(); i++) {

            double dist_to_detected_point = FloorManager.distance((float) ALLcorners.get(i).x, (float) ALLcorners.get(i).y, (float) center_point.x, (float) center_point.y);

            if (dist_to_detected_point < distance_to_center) {
                distance_to_center = dist_to_detected_point;
                the_nearest_point_index = i;
                //finding translation direction:
                if (ALLcorners.get(i).x < center_point.x) {
                    x_direction = -1;
                }

                if (ALLcorners.get(i).y < center_point.y) {
                    y_direction = -1;
                }
            }

        }

        //sorting 2 points as: left, right and store them
        corner_point_left_after_IP = new Point();
        corner_point_right_after_IP = new Point();

        if (y_direction != x_direction) {
            corner_point_right_after_IP.x = ALLcorners.get(the_nearest_point_index).x;
            corner_point_right_after_IP.y = (int) center_point.y;
            corner_point_left_after_IP.x = (int) center_point.x;
            corner_point_left_after_IP.y = ALLcorners.get(the_nearest_point_index).y;

            Imgproc.circle(dst_norm_scaled, new org.opencv.core.Point((int) center_point.x, ALLcorners.get(the_nearest_point_index).y), 1, new Scalar(255), 1); //test: for perpendicular maps orientation only
            Imgproc.circle(dst_norm_scaled, new org.opencv.core.Point(ALLcorners.get(the_nearest_point_index).x, (int) center_point.y), 3, new Scalar(255), 1); //test: for perpendicular maps orientation only

        } else {
            corner_point_left_after_IP.x = ALLcorners.get(the_nearest_point_index).x;
            corner_point_left_after_IP.y = (int) center_point.y;
            corner_point_right_after_IP.x = (int) center_point.x;
            corner_point_right_after_IP.y = ALLcorners.get(the_nearest_point_index).y;

            Imgproc.circle(dst_norm_scaled, new org.opencv.core.Point((int) center_point.x, ALLcorners.get(the_nearest_point_index).y), 3, new Scalar(255), 1); //test: for perpendicular maps orientation only
            Imgproc.circle(dst_norm_scaled, new org.opencv.core.Point(ALLcorners.get(the_nearest_point_index).x, (int) center_point.y), 1, new Scalar(255), 1); //test: for perpendicular maps orientation only
        }

        //Cont.: ..nearest detected corner to the head angle selected by the user
        if (the_nearest_point_index != -1) {
            Imgproc.circle(dst_norm_scaled, ALLcorners.get(the_nearest_point_index), 1, new Scalar(255), 1);
            corner_point_after_IP = new Point(ALLcorners.get(the_nearest_point_index).x, ALLcorners.get(the_nearest_point_index).y);
        }

        // Create bitmap
        final Bitmap bitmap = Bitmap.createBitmap(dst_norm_scaled.cols(), dst_norm_scaled.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(dst_norm_scaled, bitmap);


        return bitmap;
    }



    private Bitmap detect_corner(Bitmap resized_ip_bitmap) {

        if (FloorPlanNavigator.OpenCv_loaded) {
            Bitmap bmp32 = JPGtoRGB888(resized_ip_bitmap); //convert the image to OpenCV-suitable type

            Mat sourceImage = new Mat();
            Utils.bitmapToMat(bmp32, sourceImage);

            Bitmap bitmap_final = detectNearestCorner_Harris(sourceImage, 100);


            return bitmap_final;
        } else
            return null;
    }


    //grid around the starting pos of cam
    void make_grid(Canvas canvas) {
        if (FloorPlanNavigator.show_grid) {

            Paint blackPaint = new Paint();
            blackPaint.setColor(Color.GRAY);
            blackPaint.setAlpha(25);
            final float grid_scale = 5.62f;

            float numColumns = 115; //estimated metre according to Project Tango screen width
            float width = getWidth() * grid_scale;
            float height = getWidth() * grid_scale;

            float cellWidth = (width / numColumns);
            float cellHeight = cellWidth;


            for (int i = 1; i < numColumns; i++) {
                canvas.drawLine(i * cellWidth, 0, i * cellWidth, height, blackPaint);
                canvas.drawLine(0, i * cellHeight, width, i * cellHeight, blackPaint);
            }

        }
    }


    /**
     * Sets the new floorplan polygons model.
     */
    public void setFloorplanBoundaryObjects(List<TangoPolygon> polygons) {
        mPolygons = polygons;
    }

    public void reset_Map() {
        mPolygons.clear();
        added_points.clear();
        FloorPlanNavigator.map_calibration_phase = true;
        last_scale = 2.5f;
        last_test_scale = 2.5f;
    }

    float avg_objects_depth = 0.0f;

    /**
     * Sets the new floorplan polygons model.
     */
    public void set_Avg_ObjectsDepth_in_front_of_the_camera(float avg_depth) {
        avg_objects_depth = avg_depth;
    }

    public void registerCallback(DrawingCallback callback) {
        mCallback = callback;
    }

    /**
     * Updates the current rotation and translation to be used for the map. This is called with the
     * current device position and orientation.
     */
    public void updateCameraMatrix(float translationX, float translationY, float yawRadians, float pitchRadians) {

        if(allow_update_cam_poses) {
            mCamera.setTranslate(-translationX * SCALE, translationY * SCALE);
            mCamera.preRotate((float) Math.toDegrees(yawRadians), translationX * SCALE, -translationY * SCALE);
            mCamera.invert(mCameraInverse);

            touchRotation = (float) Math.toDegrees(yawRadians);

            float[] mat = new float[12];
            mCamera.getValues(mat);

            //get pitch rotation
            pitch = pitchRadians;
        }
    }

    //Canvas scale and translate manually
    private float[] mDispatchTouchEventWorkingArray = new float[2];
    private float[] mOnTouchEventWorkingArray = new float[2];

    //Matrices used to move and zoom image manually. Zooming is done now only automatically
    private Matrix matrix = new Matrix();
    private Matrix matrixInverse = new Matrix();
    private Matrix savedMatrix = new Matrix();

    float dx = 0;
    float dy = 0;
    float scaleFac;


    //States.
    private static final byte NONE = 0;
    private static final byte DRAG = 1;
    private static final byte ZOOM = 2;

    private byte mode = NONE;

    //Parameters for zooming.
    private PointF start = new PointF();
    private PointF mid = new PointF();
    private float oldDist = 1f;
    private float touchRotation = 0f;
    private float[] lastEvent = null;
    private long lastDownTime = 0l;

    private List<float[]> added_points = null;

    /**
     * Determine the space between the first two fingers
     */
    private float spacing(MotionEvent event) {
        float x = event.getX(0) - event.getX(1);
        float y = event.getY(0) - event.getY(1);
        return (float) Math.sqrt(x * x + y * y);
    }

    /**
     * Calculate the mid point of the first two fingers
     */
    private void midPoint(PointF point, MotionEvent event) {
        float x = event.getX(0) + event.getX(1);
        float y = event.getY(0) + event.getY(1);
        point.set(x / 2, y / 2);
    }


    private float[] scaledPointsToScreenPoints(float[] a) {
        matrix.mapPoints(a);
        return a;
    }

    private float[] screenPointsToScaledPoints(float[] a) {
        matrixInverse.mapPoints(a);
        return a;
    }


    @Override
    public boolean dispatchTouchEvent(MotionEvent ev) {
        mDispatchTouchEventWorkingArray[0] = ev.getX();
        mDispatchTouchEventWorkingArray[1] = ev.getY();
        mDispatchTouchEventWorkingArray = screenPointsToScaledPoints(mDispatchTouchEventWorkingArray);
        ev.setLocation(mDispatchTouchEventWorkingArray[0], mDispatchTouchEventWorkingArray[1]);
        return super.dispatchTouchEvent(ev);
    }


    @Override
    protected void dispatchDraw(Canvas canvas) {

        super.dispatchDraw(canvas);
    }


    @Override
    public boolean onTouchEvent(MotionEvent event) {


        if (FloorPlanNavigator.user_allows_Map) {
            if (scaleFac == 0)
                scaleFac = 1;

            float drag_speed_factor = 1;

            // handle touch events here
            mOnTouchEventWorkingArray[0] = event.getX();
            mOnTouchEventWorkingArray[1] = event.getY();
            final float density = getResources().getDisplayMetrics().density;
            mOnTouchEventWorkingArray = scaledPointsToScreenPoints(mOnTouchEventWorkingArray);

            event.setLocation(mOnTouchEventWorkingArray[0], mOnTouchEventWorkingArray[1]);

            switch (event.getAction() & MotionEvent.ACTION_MASK) {
                case MotionEvent.ACTION_DOWN:
                    savedMatrix.set(matrix);
                    savedMatrix.postRotate(touchRotation, (translationX + dx) * SCALE, (translationY + dy) * SCALE); //to move map with w.r.t touch matrix
                    mode = DRAG;
                    lastEvent = null;
                    long downTime = event.getDownTime();
                    if (downTime - lastDownTime < 300l) { //recognizing touch according to time
                        if (Math.max(Math.abs(start.x - event.getX()), Math.abs(start.y - event.getY())) < density) {
                            matrixInverse.set(matrix);
                            mid.set(event.getX(), event.getY());
                            lastEvent = new float[4];
                            lastEvent[0] = lastEvent[1] = event.getX();
                            lastEvent[2] = lastEvent[3] = event.getY();
                        }

                    } else {
                        lastDownTime = downTime;
                    }
                    start.set(event.getX() + dx / drag_speed_factor, event.getY() + dy / drag_speed_factor);
                    break;
                case MotionEvent.ACTION_POINTER_DOWN:
                    oldDist = spacing(event);
                    if (oldDist > 10f) {
                        savedMatrix.set(matrix);
                        midPoint(mid, event);
                        mode = ZOOM;
                    }
                    lastEvent = new float[4];
                    lastEvent[0] = event.getX(0);
                    lastEvent[1] = event.getX(1);
                    lastEvent[2] = event.getY(0);
                    lastEvent[3] = event.getY(1);
                    break;
                case MotionEvent.ACTION_UP:

                    mode = NONE;

                    if (Math.abs(lastDownTime - event.getEventTime()) < 90f && start != null && Math.abs(start.x - dx / drag_speed_factor - event.getX()) < 5f && Math.abs(start.y - dy / drag_speed_factor - event.getY()) < 5f) { // 50 ms click

                        float pt[] = new float[4]; //point coords (start of service coords), pose coords (middle of screen at marking moment)
                        pt[0] = dx;
                        pt[1] = dy;

                        pt[2] = FloorPlanNavigator.pose_keeper.get(FloorPlanNavigator.pose_keeper.size() - 1).getTrans_X();
                        pt[3] = FloorPlanNavigator.pose_keeper.get(FloorPlanNavigator.pose_keeper.size() - 1).getTrans_Y();

                        if (added_points.size() < 3)
                            added_points.add(pt);
                        Log.i("Click", "Click! Add point. arrSize: " + added_points.size());

                        if (added_points.size() == 1) {

                            FloorPlanNavigator.showsToastOnUiThread("Left point of corner is added.", FloorPlanNavigator.UIThread_accessibility_providing_activity);
                        } else if (added_points.size() == 2) {

                            FloorPlanNavigator.showsToastOnUiThread("Head point of corner is added, add the last point (right) to calibrate.", FloorPlanNavigator.UIThread_accessibility_providing_activity);
                        }


                    }

                    break;
                case MotionEvent.ACTION_POINTER_UP:
                    mode = NONE;
                    lastEvent = null;
                    break;
                case MotionEvent.ACTION_MOVE:
                    //matrix.set(mCamera);
                    if (mode == DRAG) {

                        matrix.set(savedMatrix);
                        dx = (-event.getX() + start.x);
                        dy = (-event.getY() + start.y);
                        matrix.postTranslate(dx, dy);
                        matrix.invert(matrixInverse);

                    } else if (mode == ZOOM) { //rotate
                        if (event.getPointerCount() > 1) {
                            float newDist = spacing(event);
                            if (newDist > density) {
                                matrix.set(savedMatrix);
                                float scale = (newDist / oldDist);
                                scaleFac = scale / 4.5f;
                                matrix.postScale(scale, scale, mid.x, mid.y);
                                touchRotation = -angleOf(event.getX(), event.getY(), mid.x, mid.y);
                                matrix.invert(matrixInverse);
                            }
                        }
                    }

                    break;
            }


            float[] values = new float[9];
            matrix.getValues(values);

            invalidate();
        }
        return true;
    }


    public float angleOf(float x1, float y1, float centre_x2, float centre_y2) {
        final double deltaY = (y1 - centre_y2);
        final double deltaX = (centre_x2 - x1);
        double result = Math.toDegrees(Math.atan2(deltaY, deltaX));
        return (result < 0) ? (float) (360f + result) : (float) result;
    }

}
