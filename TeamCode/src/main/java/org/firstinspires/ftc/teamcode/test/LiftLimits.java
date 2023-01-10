package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.LiftPID;

@TeleOp(name = "LiftLimits")
@Config()
public class LiftLimits extends LinearOpMode {
    public static int liftTarget = 20;

    private LiftPID lift = new LiftPID();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        lift.init(hardwareMap);
        lift.setLiftPosition(liftTarget);
        while (opModeIsActive() && !isStopRequested()) {
            lift.setLiftPosition(liftTarget);

            lift.updateLiftPosition();

            telemetry.addData("current ", lift.getPosition());
            telemetry.addData("target ", liftTarget);
            telemetry.addData("right ", lift.getRight());
            telemetry.addData("left ", lift.getLeft());

            telemetry.update();
        }
    }
}
