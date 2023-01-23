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
    public static int BELT_DOWN_LIMIT = -250;
    public static int BELT_UP_LIMIT = 250;// this is the difference! we go down 280, then we have to go back up 280 to get back to 0

    /*
    Claw Init Variables
     */
    public static double CLAW_CLOSE_LIMIT = .8;
    public static double CLAW_OPEN_LIMIT = .4;
}
