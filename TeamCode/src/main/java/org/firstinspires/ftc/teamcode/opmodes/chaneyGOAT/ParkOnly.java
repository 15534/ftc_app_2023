package org.firstinspires.ftc.teamcode.opmodes.chaneyGOAT;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Autonomous(name = "ParkOnly")
@Config

public class ParkOnly extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;
    Camera camera = new Camera();
    Claw claw = new Claw();
    Belt belt = new Belt();

    Pose2d startingPos = new Pose2d(36, -62, Math.toRadians(90));
    ElapsedTime runtime = new ElapsedTime();

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);

        Vector2d parkPosition = new Vector2d();
        int position = -1;
        while (!opModeIsActive() && !isStopRequested()) {
            position = camera.getPosition();
            telemetry.addData("position", position);

            telemetry.update();
            sleep(50);
        }

        if (position == 1) {
            parkPosition = new Vector2d(12, -36);
        } else if (position == 2) {
            parkPosition = new Vector2d(36, -36);
        } else if (position == 3) {
            parkPosition = new Vector2d(58, -36);
        }

        camera.webcam.stopStreaming();
        camera.webcam.stopRecordingPipeline();

        Trajectory FORWARDS =
                drive.trajectoryBuilder(startingPos)
                        .lineTo(
                            new Vector2d(36, -35),
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        ).build();

        Trajectory PARK =
                drive.trajectoryBuilder(FORWARDS.end())
                        .lineTo(
                            parkPosition
                        ).build();

        waitForStart();

        telemetry.addData("READY", "");
        telemetry.update();

        currentState = State.FORWARDS;

        while (opModeIsActive()) {
            switch (currentState) {
                case FORWARDS:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(FORWARDS);
                    }
                    next(State.PARK);
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(PARK);
                    }
                    next(State.IDLE);
                    break;
            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("current state", currentState);
            telemetry.addData("busy", drive.isBusy());
            telemetry.update();
        }

    }

    enum State {
        PARK,
        FORWARDS,
        IDLE
    }
}
