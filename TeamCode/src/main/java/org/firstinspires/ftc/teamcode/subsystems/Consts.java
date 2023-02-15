package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config("Constants")
public class Consts {
    /*
    Lift Constants
    */
    public enum Lift {
        ZERO,
        LOW,
        MEDIUM,
        HIGH,
        AUTO_GROUND,
        AUTO_LOW,
        AUTO_HIGH,
        TRANSITION
    }
    /*
    Belt constants
    */
    public enum Belt {
        UP,
        HOLD,
        DOWN,
        CONE_DROP
    }
    /*
    Claw Constants
    */
    public enum Claw {
        OPENCLAW,
        CLOSECLAW
    }
    /*
    Lift Init Variables
    */
    public static double BELT_DOWN_LIMIT = .25;
    public static double BELT_UP_LIMIT = 0.75;// this is the difference! we go down 280, then we have to go back up 280 to get back to 0
    public static double BELT_DROP_LIMIT = 0.25;

    /*
    Claw Init Variables
     */
    public static double CLAW_CLOSE_LIMIT = .52; // for slightly more precision
    public static double CLAW_OPEN_LIMIT = .88;
}
