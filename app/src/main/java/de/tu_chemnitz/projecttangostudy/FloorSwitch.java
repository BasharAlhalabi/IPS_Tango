package de.tu_chemnitz.projecttangostudy;

/**
 * Created by Halabi on 16.04.2017.
 */

public class FloorSwitch {

    public static final FloorSwitch switch_upwards   = new FloorSwitch( -1000 );
    public static final FloorSwitch switch_downwards = new FloorSwitch( -2000 );
    public static final FloorSwitch no_switch = new FloorSwitch( -3000 );

    private int _switch_indicator;

    private FloorSwitch( int s_indicator ) {
        _switch_indicator = s_indicator;
    }

    public int get_switch_indicator() {
        return _switch_indicator;
    }
}
