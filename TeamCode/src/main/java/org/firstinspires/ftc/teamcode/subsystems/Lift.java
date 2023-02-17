package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    double leftPower;
    double rightPower;
    public static int target = 0;
    public boolean requestStop = false; //dead code, but used in deprecated teleoppromax so will not delete
    public DcMotorEx left, right;
    public int mainTarget = 0;

    public void move(Consts.Lift input) {
        switch (input) {
            case ZERO:
                move(0);
                break;

            case LOW:
                move(710); // old: 780
                break;

            case MEDIUM:
                move(1255); // old 1330
                break;

            case HIGH:
                move(1820); // old 1870
                break;

            case AUTO_HIGH:
                move(1790);
                break;

            case AUTO_MEDIUM:
                move(1350);
                break;

            case AUTO_LOW:
                move(730);
                break;

            case AUTO_GROUND:
                move(100);
                break;
            case TRANSITION:
                move(500);
                break;
        }
    }

    public void move(int target) {
        setLiftPosition(target);
        mainTarget = target;
    }

    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "leftLift");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right = hardwareMap.get(DcMotorEx.class, "rightLift");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLiftPosition(int height) {
        target = height;
        double currentPos = left.getCurrentPosition();

        if (currentPos < target) {
            // Going up
            leftPower = 1;
            rightPower = -1;
        } else if (currentPos > target) {
            // Going down
            leftPower = -1;
            rightPower = 1;
        }

        left.setTargetPosition(target);
        right.setTargetPosition(-target);

        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public int getPosition() {
        // Using the average of two left and right
        return (left.getCurrentPosition() - (right.getCurrentPosition())) / 2;
    }

    public void reset() {
        move(Consts.Lift.ZERO);
    }

    public int getTarget() {
        return mainTarget;
    }
}
