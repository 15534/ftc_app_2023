package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    double leftPower;
    double rightPower;
    public static int target = 0;
    public boolean requestStop = false;
    public DcMotorEx left, right;

    public void moveLift(Constants.LiftTargets input) {
        switch (input) {
            case PICKUP:
                setLiftPosition(0);
                break;

            case LOW:
                setLiftPosition(125);
                break;

            case MEDIUM:
                setLiftPosition(325);
                break;

            case HIGH:
                setLiftPosition(575); // previous high limit: 500
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
        // Using the average of two left and right
        return (left.getCurrentPosition() - (right.getCurrentPosition())) / 2;
    }
}
