package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    }// pick up 49, 25 drop off, 310 maxheight

    public enum ClawTargets {
        OPENCLAW,
        CLOSECLAW
    }// 0, 1

    /*
    Lift Init Variables
    */
    public static double leftPowerInitial = 0.1; // counterclockwise
    public static double rightPowerInitial = -0.1; // clockwise

    public Constants(){
    }
}
