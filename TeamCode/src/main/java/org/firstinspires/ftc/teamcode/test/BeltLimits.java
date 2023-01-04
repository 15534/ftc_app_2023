package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="BeltLimits")
@Config
public class BeltLimits extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        DcMotorEx intakeLift = hardwareMap.get(DcMotorEx.class, "intakeLift");

        waitForStart();

        while (opModeIsActive()) {
            dashboardTelemetry.addData("Belt Position", intakeLift.getCurrentPosition());
            dashboardTelemetry.update();
        }
    }
}