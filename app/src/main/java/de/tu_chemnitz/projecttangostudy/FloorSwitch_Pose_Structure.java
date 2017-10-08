package de.tu_chemnitz.projecttangostudy;

/**
 * Created by Halabi on 17.04.2017.
 */

public class FloorSwitch_Pose_Structure {


    Pose_Structure pose_structure = null;
    int id_of_last_previous_pose_in_previous_Floor;
    int previous_Floor_number;
    FloorSwitch floorSwitchType= FloorSwitch.no_switch;

    public FloorSwitch_Pose_Structure(Pose_Structure pose_structure, int id_of_last_previous_pose_in_previous_Floor, int previous_Floor_number, FloorSwitch floorSwitchType)
    {
        this.pose_structure = pose_structure;
        this.id_of_last_previous_pose_in_previous_Floor = id_of_last_previous_pose_in_previous_Floor;
        this.floorSwitchType = floorSwitchType;
        this.previous_Floor_number = previous_Floor_number;
    }

    public Pose_Structure getPose_structure() {
        return pose_structure;
    }

    public int getLast_Floor_pose_id() {
        return id_of_last_previous_pose_in_previous_Floor;
    }

    public int getPrevious_Floor_number() {
        return previous_Floor_number;
    }

    public FloorSwitch getFloorSwitchType() {
        return floorSwitchType;
    }


}
