package de.tu_chemnitz.projecttangostudy;

/**
 * Created by Halabi on 16.04.2017.
 */


/**
 *    Here I create callbacks to handle switching between floors
 */
public  interface OnFloorSwitchListener {
    void OnFloorSwitch(FloorObject.FloorSwitchEvent event, int currentFloor);
}
