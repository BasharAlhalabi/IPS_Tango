package de.tu_chemnitz.projecttangostudy;

/**
 * Created by Halabi on 11.04.2017.
 */

public class Pose_Structure {


    private int pose_id = 0;
    private float trans_X = 0.0f;
    private float trans_Y = 0.0f;
    private float rotation = 0.0f;
    private int floor_number = 0;
    private boolean last_point_of_path_in_this_floor = false; //to handle multiple paths per floor (result of floor-switching)
    private boolean first_point_of_path_in_this_floor = false; //to handle multiple paths per floor (result of floor-switching)

    Pose_Structure(float x, float y)
    {
        trans_X = x;
        trans_Y = y;
        pose_id++;
    }

    public float getTrans_X() {
        return trans_X;
    }

    public float getTrans_Y() {
        return trans_Y;
    }


    public float getRotation() {
        return rotation;
    }

    public void setRotation(float rotation) {
        this.rotation = rotation;
    }

    public int getFloor_number() {
        return floor_number;
    }

    public void setFloor_number(int floor_number) {
        this.floor_number = floor_number;
    }

    public boolean isLast_point_of_path_in_this_floor() {
        return last_point_of_path_in_this_floor;
    }

    public void setLast_point_of_path_in_this_floor(boolean last_point_of_path_in_this_floor) {
        this.last_point_of_path_in_this_floor = last_point_of_path_in_this_floor;
    }

    public boolean isFirst_point_of_path_in_this_floor() {
        return first_point_of_path_in_this_floor;
    }

    public void setFirst_point_of_path_in_this_floor(boolean first_point_of_path_in_this_floor) {
        this.first_point_of_path_in_this_floor = first_point_of_path_in_this_floor;
    }

    public int getPose_id() {
        return pose_id;
    }

}
