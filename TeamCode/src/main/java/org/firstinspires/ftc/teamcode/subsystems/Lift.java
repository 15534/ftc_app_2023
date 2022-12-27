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

    private DcMotorEx left, right;

    public static boolean motorAtTarget = true;

    public static int target = 0;

    public Lift(double power) {
        power = leftPowerBase;
    }

    public void goTo(Constants.LiftTargets input) {
        switch (input) {
            case PICKUP:
                moveLift(0);
                break;

            case LOW:
                moveLift(100);
                break;

            case MEDIUM:
                moveLift(300);
                break;

            case HIGH:
                moveLift(500);
                break;

            case PUTDOWN:
                moveLift(50);
                break;
        }
    }

    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "leftLift");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        right = hardwareMap.get(DcMotorEx.class, "rightLift");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void moveLift(int height) {
        target = height;

        left.setTargetPosition((int) height);
        right.setTargetPosition((int) -height);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(leftPowerBase);
        right.setPower(rightPowerBase);
    }

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
    }
}
