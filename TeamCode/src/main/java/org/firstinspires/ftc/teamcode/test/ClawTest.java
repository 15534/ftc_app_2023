package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "ClawTest")
@Config
public class ClawTest extends LinearOpMode {
    public static double target = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Claw claw = new Claw();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        claw.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            claw.moveClaw(target);

            telemetry.addData("Claw pos", claw.getPosition());
            telemetry.update();
        }
    }
}
