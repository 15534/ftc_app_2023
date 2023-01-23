package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config("Constants")
public class Constants {
    /*
    Lift Constants
    */
    public enum LiftTargets {
        PICKUP,
        LOW,
        MEDIUM,
        HIGH,
        PUTDOWN,
    } // PICKUP: 0, LOW: 100, MEDIUM: 300, HIGH: 550, PUTDOWN: 50
    // @TODO Verify if PUTDOWN should be swapped with PICKUP

    /*
    Belt constants
    */
    public enum IntakeTargets {
        UP,
        HOLD,
        DOWN,
        CONE_DROP

    } // PICKUP: -261, HOLD: 0 , DROPOFF: -285

    /*
    Claw Constants
    */
    public enum ClawTargets {
        OPENCLAW,
        CLOSECLAW
    } // OPENCLAW: 0, CLOSECLAW: 1

    /*
    Lift Init Variables
    */
    public static int BELT_DOWN_POSITION = -250;
    public static int BELT_UP_POSITION = 250;// this is the difference! we go down 280, then we have to go back up 280 to get back to 0

    public static double clawCloseLimit = .8;
    public static double clawOpenLimit = .4;
}
