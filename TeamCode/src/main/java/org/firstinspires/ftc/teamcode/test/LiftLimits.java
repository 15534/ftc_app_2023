package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "LiftLimits")
@Config
public class LiftLimits extends LinearOpMode {

    public static int target = 0;

    private Lift lift = new Lift();
    private Belt belt = new Belt();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift.init(hardwareMap);
        belt.init(hardwareMap);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lift.moveLift(target);
            belt.moveBeltNoCorrection(Constants.IntakeTargets.DOWN);
            telemetry.addData("Lift Position ", lift.getPosition());
            telemetry.addData("Lift Position ", belt.getPosition());
            telemetry.update();
        }
    }
}
