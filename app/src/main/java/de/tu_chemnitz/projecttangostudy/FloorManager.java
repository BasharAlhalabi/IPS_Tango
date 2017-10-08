package de.tu_chemnitz.projecttangostudy;

import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import com.google.atap.tango.reconstruction.Tango3dReconstruction;
import com.google.atap.tango.reconstruction.Tango3dReconstructionConfig;
import com.google.atap.tango.reconstruction.TangoPolygon;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;

import java.nio.FloatBuffer;
import java.util.List;

/**
 * Created by Bashar Al Halabi on 12.04.2017.
 * A middleware between raw pos, rot. It does the needed calculations and provides them to the renderer
 * Idea: from com.projecttango.examples.java.floorplanreconstruction
 * Note: Calculation style does not maintain restrictions of solid OOP principles intentionally at the time being for experimental purposes
 */

public class FloorManager extends Tango.TangoUpdateCallback{

    private static final String TAG = FloorPlanNavigator.class.getSimpleName();

    //Global variables-----------------------
    private final TangoPointCloudManager mPointCloudBuffer;

    private FloorObject mFloorObject = null; //to handle events of floor change
    private Tango3dReconstruction mTango3dReconstruction = null;
    private OnFloorPlanDataAvailableListener mFloorPlanDataAvailableCallback = null;
    private HandlerThread mHandlerThread = null;
    private volatile Handler mHandler = null;
    private volatile boolean mIsFloorplanningActive = false;

    private Runnable mRunnableCallback = null;
    //End Global variables-----------------------

    /**
     * Callback for when meshes are available.
     */
    interface OnFloorPlanDataAvailableListener {
        void onFloorPlanDataAvailable(List<TangoPolygon> polygons);
    }

    /**
     * Synchronize access to mTango3dReconstruction. This runs in UI thread.
     */
    synchronized void release() {
        mIsFloorplanningActive = false;
        mTango3dReconstruction.release();
    }

    void startFloorplanning() {
        mIsFloorplanningActive = true;
    }

    void stopFloorplanning() {
        mIsFloorplanningActive = false;
    }

    /**
     * Synchronize access to mTango3dReconstruction. This runs in UI thread.
     */
    synchronized void resetFloorplan() {
        if(mTango3dReconstruction != null)
            mTango3dReconstruction.clear();
    }



    /**
     * Synchronize access to mTango3dReconstruction. This runs in UI thread.
     */
    synchronized void setDepthCameraCalibration(TangoCameraIntrinsics calibration) {
        mTango3dReconstruction.setDepthCameraCalibration(calibration);
    }



    FloorManager(OnFloorPlanDataAvailableListener callback)
    {

        mFloorPlanDataAvailableCallback = callback;
        Tango3dReconstructionConfig config = new Tango3dReconstructionConfig();
        // Configure the 3D reconstruction library to work in "floorplan" mode.
        config.putBoolean("use_floorplan", true);
        config.putBoolean("generate_color", false);
        // Simplify the detected countours by allowing a maximum error of (5cm --> 1cm)
        config.putDouble("floorplan_max_error", 0.05);
        mTango3dReconstruction = new Tango3dReconstruction(config);

        mPointCloudBuffer = new TangoPointCloudManager();

        mHandlerThread = new HandlerThread("mesherCallback");
        mHandlerThread.start();
        mHandler = new Handler(mHandlerThread.getLooper());

        if (callback != null) {
            /**
             * This runnable processes the saved point clouds and meshes and triggers the
             * onFloorplanAvailable callback with the generated {@code TangoPolygon} instances.
             */
            mRunnableCallback = new Runnable() {
                @Override
                public void run() {
                    // Synchronize access to mTango3dReconstruction. This runs in FloorManager thread.
                    synchronized (FloorManager.this) {
                        if (!mIsFloorplanningActive) {
                            return;
                        }

                        if (mPointCloudBuffer.getLatestPointCloud() == null) {
                            return;
                        }

                        // Get the latest point cloud data.
                            TangoPointCloudData cloudData = mPointCloudBuffer.getLatestPointCloud();

                            TangoPoseData depthPose = null;

                        if(!FloorPlanNavigator.isDriftCorrection && !FloorPlanNavigator.isLearningMode)
                            depthPose = TangoSupport.getPoseAtTime(cloudData.timestamp,
                                    TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                    TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                    TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                    TangoSupport.ROTATION_IGNORED);
                            else
                            depthPose = TangoSupport.getPoseAtTime(cloudData.timestamp,
                                    TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                                    TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                    TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                    TangoSupport.ROTATION_IGNORED);

                            if (depthPose.statusCode != TangoPoseData.POSE_VALID) {
                                Log.e(TAG, "couldn't extract a valid depth pose");
                                return;
                            }

                        // Update the mesh and floorplan representation.
                        mTango3dReconstruction.updateFloorplan(cloudData, depthPose);

                        // Extract the full set of floorplan polygons.
                        List<TangoPolygon> polygons = mTango3dReconstruction.extractFloorplan();

                        // Provide the new floorplan polygons to the app via callback.
                        mFloorPlanDataAvailableCallback.onFloorPlanDataAvailable(polygons);
                    }
                }
            };
        }

        mFloorObject = new FloorObject();
        if(FloorPlanNavigator.m2D_RenderingView != null)
            mFloorObject.addFloorSwitchListener(FloorPlanNavigator.m2D_RenderingView);
    }


     void setAvailablePose(TangoPoseData pose) { //here I calculate the floor number based on Y and raise an event about floor switch

        float translation[] = pose.getTranslationAsFloats(); //Y: translation[1]
        handle_floor_switching(translation[1]);

    }



    private void handle_floor_switching(float cameraY)
    {

        final double floorDeck_uncertainty = 0.1;
        final double handHeight_uncertainty = 0.3;
        final double buffer_Interval = handHeight_uncertainty + floorDeck_uncertainty;

        double floor_ground = cameraY - FloorPlanNavigator.handHeight + buffer_Interval; //Y=Floor of floor (m) test as --------Buffer_Interval /2 with ,475 as val: HH + FDk

        boolean floor_switched = false;
        int prevFloorNum = FloorPlanNavigator.currFloorNum;

        //calculate the current floor
        if(!FloorPlanNavigator.different_floors_heights)
        {
            FloorPlanNavigator.currFloorNum = FloorPlanNavigator.startFloorNum +  (int)(Math.ceil((floor_ground)/ FloorPlanNavigator.floorHeight));
            //calculate rest height to switch
            FloorPlanNavigator.height_to_switch = Math.abs(floor_ground/ FloorPlanNavigator.floorHeight);
        }
        /*else if(FloorPlanNavigator.current_floor_number >= 0)
        {
            FloorPlanNavigator.current_floor_number = FloorPlanNavigator.StartFloorNum +  (int)(Math.ceil((floor_ground)/ FloorPlanNavigator.estimated_floor_heightS[FloorPlanNavigator.current_floor_number]));
            //calculate rest height to switch
            FloorPlanNavigator.height_to_switch = Math.abs(floor_ground/ FloorPlanNavigator.estimated_floor_heightS[FloorPlanNavigator.current_floor_number]);
        }*/

        //restrict height to switch between 0 and 1 (undirect normalization)
        if(FloorPlanNavigator.height_to_switch > 1)
            FloorPlanNavigator.height_to_switch--;
        else if(FloorPlanNavigator.height_to_switch < 0)
            FloorPlanNavigator.height_to_switch++;

        //check if floor is just changed
        if(prevFloorNum < FloorPlanNavigator.currFloorNum) {
            floor_switched = true;
            mFloorObject.receiveSwitchedUpwards();
        }
        else if(prevFloorNum > FloorPlanNavigator.currFloorNum) {
            floor_switched = true;
            mFloorObject.receiveSwitchedDownwards();
        }

        if(floor_switched)
            reset_floor_height_rt_calculating();
    }


    private void reset_floor_height_rt_calculating()
    {
        //lowest_y_acc = Float.MAX_VALUE; //exchanged
        //highest_y_acc = Float.MIN_VALUE; //exchanged
        lowest_y = Float.MAX_VALUE;
        highest_y = Float.MIN_VALUE;
        FloorPlanNavigator.realTime_height = 0;
    }

   //float lowest_y_acc = Float.MAX_VALUE; //min detected height over time //exchanged
   //float highest_y_acc = Float.MIN_VALUE; //max detected height over time //exchanged

    static float[] current_corresponding_point = {0, 0}; //max depth point (walls corner)
    static float[] Pz = new float[2]; //max depth point (walls corner)
    static float[] Pr1 = new float[2]; //a left point of a corner
    static float[] Pr2 = new float[2]; //a right point of a corner

    static float lowest_y = Float.MAX_VALUE; //curr min height
    static float highest_y = Float.MIN_VALUE; //cur max height

    final float perpendicular_cut_to_cam_vector_m = 1.0f; //line connects right-left real points at 1 meter from corner, perpendicular to cam-view-vector


    //find out height
    static boolean camera_corner_pos_initialized = false;

    double calculated_camera_height = 0.0;
    static boolean angle_between_3p_is_right = false;

    /**
     * Calculates the average depth from a point cloud buffer.
     */
    private float calculateAveragedDepth_and_do_experiments(FloatBuffer pointCloudBuffer, int numPoints) {


        float furthest_z_general_m = Float.MIN_VALUE;
        float x_of_furthest_z_general = 0;

        lowest_y = Float.MAX_VALUE;
        highest_y = Float.MIN_VALUE;
        float max_right_x = Float.MIN_VALUE;
        float max_left_x = Float.MAX_VALUE;
        float totalZ = 0;
        float averageZ = 0;

        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {

                //avg calculation step 1
                totalZ = totalZ + pointCloudBuffer.get(i);

/*                // accumulative RT height calculation over time //exchanged
                if(!camera_corner_pos_initialized) {
                    if (pointCloudBuffer.get(i - 1) < lowest_y_acc) {
                        lowest_y_acc = pointCloudBuffer.get(i - 1);
                    }
                    if (pointCloudBuffer.get(i - 1) > highest_y_acc) {
                        highest_y_acc = pointCloudBuffer.get(i - 1);
                    }
                }
*/

                //  current RT height calculation:
                if (pointCloudBuffer.get(i - 1) < lowest_y) {
                    lowest_y = pointCloudBuffer.get(i - 1);
                }
                if (pointCloudBuffer.get(i - 1) > highest_y) {
                    highest_y = pointCloudBuffer.get(i - 1);
                }

                //deepest point w.r.t camera
                if (.3 >= pointCloudBuffer.get(i - 2) && -.3 <= pointCloudBuffer.get(i - 2)) //X range to searching for max depth (middle of cam view vertically)
                {
                    if (pointCloudBuffer.get(i) > furthest_z_general_m) {
                        furthest_z_general_m = pointCloudBuffer.get(i);
                        x_of_furthest_z_general = pointCloudBuffer.get(i - 2);
                    }

                    if (.01 >= pointCloudBuffer.get(i - 2) && -.01 <= pointCloudBuffer.get(i - 2)) {
                        current_corresponding_point[1] = pointCloudBuffer.get(i);
                    }
                }

            }
            averageZ = totalZ / numPoints;

/*            //accumulative modded max
            if(head_max_depth_reads_cm.size() > 10 && false) { //finding centroid. Exchanged

                furthest_z_general_m = mode(convertIntegers(head_max_depth_reads_cm));

                furthest_z_general_m /= 1000.0f; // to meter
            }
*/
            //form a vertical corner between 2 walls with error range of 2 cm
            for (int i = 2; i < numFloats; i = i + 4) {

                if (pointCloudBuffer.get(i - 2) > max_right_x && (furthest_z_general_m - perpendicular_cut_to_cam_vector_m) + 0.01 >= pointCloudBuffer.get(i) && (furthest_z_general_m - perpendicular_cut_to_cam_vector_m) - 0.01 <= pointCloudBuffer.get(i))
                {
                    max_right_x = pointCloudBuffer.get(i - 2);
                }
                if (pointCloudBuffer.get(i - 2) < max_left_x && (furthest_z_general_m - perpendicular_cut_to_cam_vector_m) + 0.01 >= pointCloudBuffer.get(i) && (furthest_z_general_m - perpendicular_cut_to_cam_vector_m) - 0.01 <= pointCloudBuffer.get(i))
                {
                    max_left_x = pointCloudBuffer.get(i - 2);
                }
            }

/*            //accumulative modded maxes
            if(left_max_depth_reads_cm.size() > 10 && right_max_depth_reads_cm.size() > 10 && false) { //finding centroid. Exchanged

                max_left_x = (float) mode( convertIntegers(left_max_depth_reads_cm));
                max_right_x = (float) mode( convertIntegers(right_max_depth_reads_cm));

                max_left_x /= 1000.0f; // to meter
                max_right_x /= 1000.0f; // to meter
            }
*/

            //deep point
            Pz[0] = x_of_furthest_z_general; //2d: x
            Pz[1] = furthest_z_general_m; //tief point //2d: y //3d: z

            //left point
            Pr1[0] = max_left_x;
            Pr1[1] = (furthest_z_general_m - perpendicular_cut_to_cam_vector_m);

            //right point
            Pr2[0] = max_right_x;
            Pr2[1] = (furthest_z_general_m - perpendicular_cut_to_cam_vector_m);

            float angle_between_3p = angle_between_3_points_wrt_first_one(Pz[0], Pz[1], Pr1[0], Pr1[1], Pr2[0], Pr2[1]);

            //check if corner makes since, with error range of 1 degree:
            angle_between_3p_is_right = Math.abs(angle_between_3p) > 89.5f && Math.abs(angle_between_3p) < 90.5f && furthest_z_general_m != Float.MIN_VALUE;

        }


        calculated_camera_height = Math.abs(lowest_y); //originally but exchanged: findMax(hand_heights);

        //FloorPlanNavigator.realTime_height = Math.abs(highest_y_acc - lowest_y_acc); // accumulative over time //exchanged
        FloorPlanNavigator.realTime_height = Math.abs(highest_y - lowest_y); //directly calculated from the current point cloud only
        FloorPlanNavigator.realTime_Hands_height = calculated_camera_height;//dist(pos_of_point_by_nearest_z[0], pos_of_point_by_nearest_z[1], pos_of_point_by_furthest_z[0], pos_of_point_by_furthest_z[1], pos_of_point_by_nearest_z[2], pos_of_point_by_furthest_z[2]);//Math.abs(pos_of_point_by_furthest_z[1] - pos_of_point_by_nearest_z[1]) ; //test

        return averageZ;
    }


    public static int mode(int[] input) {

        int[] count = new int[input.length];

        //count the occurrences
        for (int i=0; i < input.length; i++) {
            count[input[i]]++;
        }

        //go backwards and find the count with the most occurrences
        int index = count.length-1;
        for (int i=count.length-2; i >=0; i--) {
            if (count[i] >= count[index])
                index = i;
        }

        return index;
    }


    static public float angle_between_3_points_wrt_first_one(float x1, float y1, float x2, float y2, float x3, float y3) {

        float teta = 0.0f;
        
        double var1 = Math.atan2(y1 - y2, x1 - x2);
        double var2 = Math.atan2(y1 - y3, x1 - x3);
        teta = (float) Math.toDegrees(var1-var2);

        return teta;
    }


    //find point on line with distance from first given point
    static public float[] point_on_line_with_length_ratio_starting_from_first_point_using_points(float length_m, float x1, float y1, float x2, float y2) {

        float[] point = new float[2];

        //length should be from 0 to 1:
        float ratio = length_m/(float) distance(x1, y1, x2, y2) ;

        point[0] = ratio * x1 + (1.0f - ratio) * x2;
        point[1] = ratio * y1 + (1.0f - ratio) * y2;

        return point;
    }


    //find point on line with a fixed distance from first given point
    static float[] point_on_line_with_fixed_length_from_first_point(float length_m, float x1, float y1, float x2, float y2) {

        float[] point = new float[2];

        point[0] = (float) (x1 + length_m * ( (x2 - x1) / (Math.sqrt( Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2) )) ));
        point[1] = (float) (y1 + length_m * ( (y2 - y1) / (Math.sqrt( Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2) )) ));
        return point;
    }


    static double distance(float X1, float Y1, float X2, float Y2) {  //2d in m

        double distance = 0;

        distance = Math.sqrt(Math.pow((X1 - X2), 2) + Math.pow((Y1 - Y2), 2));

        return distance;

    }

    private double findMax(double... vals) {
        double max = Double.NEGATIVE_INFINITY;

        for (double d : vals) {
            if (d > max) max = d;
        }

        return max;
    }

    private double distance(float X1, float Y1, float X2, float Y2, float Z1, float Z2) { //3d in m

        double distance = 0;

        distance = Math.sqrt(Math.pow((X1 - X2), 2) + Math.pow((Y1 - Y2), 2) + Math.pow((Z1 - Z2), 2));

        return distance;
    }


    //Overrides:
    @Override
    public void onPoseAvailable(TangoPoseData pose) {
        //do nothing. Pose is taken from the listener placed in FloorPlanNavigator directly to _2D_RenderingView
    }

    @Override
    public void onXyzIjAvailable(final TangoXyzIjData var1) {
        // do nothing.
    }

    /**
     * Receives the depth point cloud. This method retrieves and stores the depth camera pose
     * and point cloud to later use it when updating the {@code Tango3dReconstruction}.
     *
     * @param tangoPointCloudData the depth point cloud.
     */
    @Override
    public void onPointCloudAvailable(final TangoPointCloudData tangoPointCloudData) {

            if (!mIsFloorplanningActive || tangoPointCloudData == null || tangoPointCloudData.points == null) {
                return;
            }

            //calculate avg depth
            final float avg_depth = calculateAveragedDepth_and_do_experiments(tangoPointCloudData.points, tangoPointCloudData.numPoints);
            //pass avg_depth to the renderer
            FloorPlanNavigator.m2D_RenderingView.set_Avg_ObjectsDepth_in_front_of_the_camera(avg_depth);

            //display the result as Text in the dirty way:
            FloorPlanNavigator.avg_depth = avg_depth;

            //log avg_depth
            if (FloorPlanNavigator.show_avg_depth_log)
                logPointCloud(avg_depth, tangoPointCloudData.numPoints);

            mPointCloudBuffer.updatePointCloud(tangoPointCloudData);

            mHandler.removeCallbacksAndMessages(null);
            mHandler.post(mRunnableCallback);

    }



    @Override
    public void onFrameAvailable(int var1) {
        // do nothing.
    }

    @Override
    public void onTangoEvent(TangoEvent var1) {
        // do nothing.
    }


    /**
     * Log the point count and the average depth of the given PointCloud data
     * in the Logcat.
     */
    private void logPointCloud(float avg_depth, int points_number) {

        String stringBuilder = "Point count: " + points_number + ". Average depth (m): " + avg_depth;
        Log.i(TAG, stringBuilder);
    }


}
