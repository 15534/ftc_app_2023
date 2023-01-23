package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Belt;

@TeleOp(name = "BeltLimits")
@Config
public class BeltLimits extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Belt belt = new Belt();

        waitForStart();

        belt.init(hardwareMap);

        while (opModeIsActive()) {
            dashboardTelemetry.addData("Belt Position", belt.getPosition());
            dashboardTelemetry.update();
        }
    }
}
