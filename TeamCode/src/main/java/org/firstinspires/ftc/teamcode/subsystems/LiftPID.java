package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PID;

// @TODO: Verify if the file is still needed

public class LiftPID {
    public static double leftPowerBase = 50;
    public static double rightPowerBase = -leftPowerBase;
    public static boolean motorAtTarget = true;
    public static int target = 0;
    public boolean requestStop = false;
    public static double baseGain = -.01;
    public static double negativeGain = -.002;
    private double gain;
    public DcMotorEx left, right;
    private Constants.LiftTargets targetEnum;
    private PID pid = new PID();
    private ElapsedTime pidTimer = new ElapsedTime();
    private double pidOut;
    private boolean canMove = true;

    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "leftLift");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right = hardwareMap.get(DcMotorEx.class, "rightLift");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pid.init(pidTimer);

        //        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftPosition(int height) {
        target = height;
        pid.setTarget(target);
    }

    public void updateLiftPosition() {
        pidOut = pid.update(getPosition()); // set curr pos
        left.setPower(pidOut);
        right.setPower(-1 * pidOut);
    }

    public int getPosition() {
        return ((left.getCurrentPosition() - right.getCurrentPosition()) / 2);
        //        return left.getCurrentPosition();
    }

    public int getRight() {
        return right.getCurrentPosition();
    }

    public int getLeft() {
        return left.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }
}
