package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.subsystems.Consts.CLAW_CLOSE_LIMIT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ClawLimits")
public class ClawLimits extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo clawServo = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            clawServo.setPosition(CLAW_CLOSE_LIMIT);

            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.update();
        }
    }
}
