package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "LiftLimits")
@Config()
public class LiftLimits extends LinearOpMode {

    public static double leftPowerInitial = 0.1; // counterclockwise
    public static double rightPowerInitial = -0.1; // clockwise

    public static double leftPos = 20; // counterclockwise
    public static double rightPos = -20; // clockwise

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "leftLift");
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "rightLift");
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            if (left.getCurrentPosition() > leftPos - 5
                    && left.getCurrentPosition() < leftPos + 5) {
                double leftPower = left.getPower();
                double rightPower = right.getPower();
                while (leftPower >= 0.02) { // quick fix - write better, smoother function later
                    left.setPower(leftPower - leftPower / 5);
                    right.setPower(
                            rightPower
                                    + rightPower
                                            / 5); // power same for both, just opposite direction
                }
                left.setPower(0); // full break
                right.setPower(0); // full break

            } else {
                left.setTargetPosition((int) leftPos);
                right.setTargetPosition((int) rightPos);

                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                left.setPower(leftPowerInitial);
                right.setPower(rightPowerInitial);
            }

            telemetry.addData("Left: ", left.getCurrentPosition());
            telemetry.addData("Right: ", right.getCurrentPosition());

            telemetry.update();
        }
    }
}
