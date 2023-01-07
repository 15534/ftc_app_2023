package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.C;

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
        PUTDOWN
    }

    public enum IntakeTargets {
        PICKUP,
        HOLD,
        DROPOFF
    }// -261 pick up, 0 hold, -285 drop off

    public enum ClawTargets {
        OPENCLAW,
        CLOSECLAW
    }// 0, 1



    /*
    Lift Init Variables
    */
    public static double leftPowerInitial = 0.1; // counterclockwise
    public static double rightPowerInitial = -0.1; // clockwise

    public static int BELT_POSITION_START = -280;
    public static int BELT_POSITION_END = 280; // this is the difference! we go down 280, then we have to go back up 280 to get back to 0
//    public static int BELT_POSITION_UP = ;


    public Constants(){
    }
}
