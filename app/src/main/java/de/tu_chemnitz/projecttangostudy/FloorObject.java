package de.tu_chemnitz.projecttangostudy;

import java.util.ArrayList;
import java.util.EventObject;
import java.util.Iterator;
import java.util.List;

/**
 * Created by Halabi on 16.04.2017.
 */


    /*
    * Here I create callbacks to handle switching between floors
    * */


public class FloorObject {

    private FloorSwitch switch_type = FloorSwitch.no_switch; //init
    private List _listeners = new ArrayList();


    public class FloorSwitchEvent extends EventObject {
        private FloorSwitch switch_type;

        public FloorSwitchEvent( Object source, FloorSwitch switch_type ) {
            super( source );
            this.switch_type = switch_type;
        }

        public FloorSwitch switchType() {
            return switch_type;
        }

        public String toString() {
            if(switch_type == FloorSwitch.switch_downwards)
                return "Floor Changed Downwards";
            else if(switch_type == FloorSwitch.switch_upwards)
                return "Floor Changed Upwards";
            else return "Sth. is wrong with Floor Switching callbacks";
        }
    }


    public synchronized void receiveSwitchedUpwards() {

            switch_type = FloorSwitch.switch_upwards;
            _fireFloorSwitchEvent();

    }
    public synchronized void receiveSwitchedDownwards() {

            switch_type = FloorSwitch.switch_downwards;
            _fireFloorSwitchEvent();

    }
    public synchronized void addFloorSwitchListener( OnFloorSwitchListener fs ) {
        _listeners.add( fs );
    }

    public synchronized void removeMoodListener( OnFloorSwitchListener fs ) {
        _listeners.remove( fs );
    }

    private synchronized void _fireFloorSwitchEvent() {
        FloorSwitchEvent FSE = new FloorSwitchEvent( this, switch_type );
        Iterator listeners = _listeners.iterator();
        while( listeners.hasNext() ) {
            ( (OnFloorSwitchListener) listeners.next() ).OnFloorSwitch( FSE , FloorPlanNavigator.currFloorNum);
        }
    }
}
