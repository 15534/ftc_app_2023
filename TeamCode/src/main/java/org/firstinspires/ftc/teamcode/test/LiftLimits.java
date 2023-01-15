package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "LiftLimits")
@Config
public class LiftLimits extends LinearOpMode {

    public static int target = 0;

    private Lift lift = new Lift();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lift.moveLift(target);

            telemetry.addData("Lift Position ", lift.getPosition());
            telemetry.update();
        }
    }
}
