package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.clawCloseLimit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ClawLimits")

public class ClawLimits extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo clawServo = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            clawServo.setPosition(clawCloseLimit);

            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.update();
        }
    }
}
