package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    double leftPower;
    double rightPower;
    public static boolean motorAtTarget = true;
    public static int target = 0;
    public boolean requestStop = false;
    public static double baseGain = -.01;
    public static double negativeGain = -.002;
    private double gain;
    public DcMotorEx left, right;
    private Constants.LiftTargets targetEnum;

    public void moveLift(Constants.LiftTargets input) {
        targetEnum = input;
        switch (input) {
            case PICKUP:
                setLiftPosition(0);
                break;

            case LOW:
                setLiftPosition(100);
                break;

            case MEDIUM:
                setLiftPosition(300);
                break;

            case HIGH:
                setLiftPosition(550); // previous high limit: 500
                break;

            case PUTDOWN:
                setLiftPosition(50);
                break;
        }
    }

    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "leftLift");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right = hardwareMap.get(DcMotorEx.class, "rightLift");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // just setting position, not actually moving there - no moving during init!
    }

    public void setLiftPosition(int height) {
        target = height;
        double currentPos = left.getCurrentPosition();

        if (currentPos < target) {
            // going up
            leftPower = 1;
            rightPower = -1;
        } else if (currentPos > target) {
            // going down
            leftPower = -0.5;
            rightPower = 0.5;
        }

        left.setTargetPosition((int) target);
        right.setTargetPosition((int) -target);

        left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public int getPosition() {
        return (left.getCurrentPosition() - (right.getCurrentPosition())) / 2;
    }

    /*
    public void updateLiftPosition() {
        if (targetEnum == Constants.LiftTargets.PICKUP) {

            // start easing 100 ticks away
            // start at 500, go to 0
            // Reaches 104, sets position to 0
            // ??
            if (left.getCurrentPosition()<100){

                double newPower = (left.getCurrentPosition() - target) * gain;
                right.setPower(-newPower);
                setLiftPosition(0);
            }
            else{
                setLiftPosition(100);
            }

        }

        // safety, in case lift is doing something unreasonable
        if (left.getCurrentPosition() > 600 || left.getCurrentPosition() < 0) {
            left.setPower(0);
            right.setPower(0);
            requestStop = true;
        } else if (Math.abs(target - left.getCurrentPosition()) > 4) {
            if (left.getCurrentPosition() - target > 0) {
                gain = negativeGain;
            } else {
                gain = baseGain;
            }
            double newPower = (left.getCurrentPosition() - target) * gain;

            left.setPower(newPower); // positive if below target, negative if above target


        }
    }

x    /*
    old update function - git blame: riya
    public void update() {
        if (left.getCurrentPosition() > target - 5 && left.getCurrentPosition() < -target + 5) {
            motorAtTarget = false;

            double leftPower = left.getPower();
            double rightPower = right.getPower();
            if (leftPower >= 0.02) {
                left.setPower(leftPower - leftPower / 5);
                right.setPower(
                        rightPower
                                + rightPower / 5); // power same for both, just opposite direction
            }
            left.setPower(0); // full brake
            right.setPower(0); // full brake
        } else {
            motorAtTarget = true;
        }
    } */
}

