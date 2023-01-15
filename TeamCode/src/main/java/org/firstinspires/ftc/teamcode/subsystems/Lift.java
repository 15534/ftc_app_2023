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
    public int mainTarget = 0;

    public void moveLift(Constants.LiftTargets input) {
        switch (input) {
            case PICKUP:
                setLiftPosition(0);
                break;

            case LOW:
                setLiftPosition(300);
                break;

            case MEDIUM:
                setLiftPosition(400); 
                break;

            case HIGH:
                setLiftPosition(695); // previous high limit: 500
                break;

            case PUTDOWN:
                setLiftPosition(50);
                break;

            case AUTO_CONE_1:
                setLiftPosition(100);
                break;
            case AUTO_CONE_2:
                setLiftPosition(80);
                break;
            case AUTO_CONE_3:
                setLiftPosition(60);
                break;
            case AUTO_CONE_4:
                setLiftPosition(30);
                break;
            case AUTO_CONE_5:
                setLiftPosition(0);
                break;
        }
    }

    public void moveLift(int target) {
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

    public int getTarget(){
        return mainTarget;
    }
}
