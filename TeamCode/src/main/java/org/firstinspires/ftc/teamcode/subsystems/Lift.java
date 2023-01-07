package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.LiftTargets.PICKUP;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public static double leftPowerBase = 50;
    public static double rightPowerBase = -leftPowerBase;
    public static boolean motorAtTarget = true;
    public static int target = 0;
    public boolean requestStop = false;
    double baseGain = -.01;
    double negativeGain = -.002;
    private double gain;
    public DcMotorEx left, right;
    private Constants.LiftTargets targetEnum;

    public static boolean motorAtTarget = true;

    public static int target = 0;

    public void moveLift(Constants.LiftTargets input) {
        targetEnum = input;
        switch (input) {
            case PICKUP:
//                setLiftPosition();
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

        moveLift(Constants.LiftTargets.PICKUP);
        // just setting position, not actually moving there - no moving during init!
    }

    public void setLiftPosition(int height) {
        target = height;

        /* left.setTargetPosition((int) height);
        right.setTargetPosition((int) -height);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(leftPowerBase);
        right.setPower(rightPowerBase); */
    }

    public void updateLiftPosition() {
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
            right.setPower(-newPower);
        }
    }

    public int getPosition() {
        return left.getCurrentPosition();
    }

    /*
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
