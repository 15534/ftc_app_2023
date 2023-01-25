package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Belt;

@TeleOp(name = "BeltLimits")
@Config("BeltLimits")
public class BeltLimits extends LinearOpMode {
    public static double beltTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Belt belt = new Belt();

        waitForStart();

        belt.init(hardwareMap);

        while (opModeIsActive()) {
            belt.moveAbsolute(beltTarget);
            telemetry.update();
        }
    }
}
