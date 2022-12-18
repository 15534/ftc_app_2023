package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="beltLimits")
@Config
public class beltLimits extends LinearOpMode {

    public static double powerInitial = 0.05;
    public static double position = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intakeLift = hardwareMap.get(DcMotorEx.class, "intakeLift");

        waitForStart();

        while (opModeIsActive()) {

            // intakeLift.setTargetPosition((int) position);
            // intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // intakeLift.setPower(powerInitial);

            telemetry.addData("IntakeLiftPos", intakeLift.getCurrentPosition());
            telemetry.update();
        }
    }
}

