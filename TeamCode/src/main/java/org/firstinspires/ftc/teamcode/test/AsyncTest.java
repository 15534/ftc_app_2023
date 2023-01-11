package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@TeleOp(name = "AsyncTest")
@Config
public class AsyncTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnTable ourTurntable = new TurnTable();
        ourTurntable.init(hardwareMap);

        Trajectory trajectoryForward =
                drive.trajectoryBuilder(new Pose2d())
                        .forward(20)
                        .addDisplacementMarker(() -> ourTurntable.turn(180))
                        .build();
        Trajectory trajectoryBack = drive.trajectoryBuilder(new Pose2d()).back(13).build();

        waitForStart();
        drive.followTrajectory(trajectoryForward);

        // @TODO: Verify dead code
        while (drive.isBusy()) {
            // ourTurntable.turn((180));
            continue;
        }

        drive.followTrajectory(trajectoryBack);

        while (opModeIsActive() && !isStopRequested()) {}
    }
}
