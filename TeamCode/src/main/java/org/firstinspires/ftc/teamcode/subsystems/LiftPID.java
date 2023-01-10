package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PID;

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


    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "leftLift");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right = hardwareMap.get(DcMotorEx.class, "rightLift");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pid.init(pidTimer, 0.1, 0.05, 0);

    }

    public void setLiftPosition(int height) {
        target = height;
        pid.setTarget(target);
    }

    public void updateLiftPosition() {
        pidOut = pid.update(left.getCurrentPosition());
        left.setPower(pidOut);
        right.setPower(-1*pidOut);
    }

    public int getPosition() {
        return left.getCurrentPosition();
    }

    public int getTarget(){return target;}

}
