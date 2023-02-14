package org.firstinspires.ftc.teamcode.opmodes.experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "PathsOnlyAutoEx")
@Config
public class Experimental_PathsOnlyAuto extends LinearOpMode {

    SampleMecanumDrive drive;

    enum State {
        PATH
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startingPos = new Pose2d(40, -62, Math.toRadians(90));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Trajectory TEST_TRAJECTORY = drive.trajectoryBuilder(startingPos)
                .lineToSplineHeading(new Pose2d(37, -5, Math.toRadians(-45)))
                .build();

        waitForStart();

        State currentState = State.PATH;

        while (opModeIsActive()) {
            switch (currentState) {
                case PATH:
                    if (!drive.isBusy()) {
                    drive.followTrajectory(TEST_TRAJECTORY);
                    drive.update();
                    drive.updatePoseEstimate();
                    }
                    break;
            }
            telemetry.update();
        }
    }
}
