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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "1+0_RIGHT", group = "ChaneyTheGOAT")
@Config

public class OnePlusZero_Right extends LinearOpMode {

    enum State {
        HIGH_POLE,
        PARK,
        IDLE
    }

    SampleMecanumDrive drive;

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    Camera camera = new Camera();
    TurnTable turnTable = new TurnTable();

    State currentState = State.IDLE;
    ElapsedTime runTime = new ElapsedTime();
    int maxVelocity = 60;
    int maxAccel = 60;

    Pose2d startingPos = new Pose2d(40, -62, Math.toRadians(90));
    Vector2d parkPosition = new Vector2d();

    void next(State s) {
        time = runTime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turnTable.init(hardwareMap);
        lift.init(hardwareMap);

        // Trajectories
        TrajectorySequence highPole =
                drive.trajectorySequenceBuilder(startingPos)
                        .splineToConstantHeading(
                                new Vector2d(34.56, -53.17), Math.toRadians(90.00),
                                SampleMecanumDrive.getVelocityConstraint(maxVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                        )
                        .splineToConstantHeading(new Vector2d(35.94, -29.97), Math.toRadians(90.00))
                        .splineTo(new Vector2d(30.5, -10.75), Math.toRadians(125.00))

                        .addSpatialMarker(new Vector2d(35, -50),
                                () -> {
                                    lift.move(Consts.Lift.AUTO_HIGH);
                                })
                        .addSpatialMarker(new Vector2d(36, -14),
                                () -> {
                                    belt.move(Consts.Belt.CONE_DROP);
                                })
                        .build();

        // Camera + Park
        runTime.reset();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("position", camera.getPosition());
            telemetry.update();

            parkPosition = new Vector2d(36, -12);
            switch (camera.getPosition()) {
                case 1:
                    parkPosition = new Vector2d(12, -12);
                    break;
                case 2:
                    parkPosition = new Vector2d(36, -12);
                    break;
                case 3:
                    parkPosition = new Vector2d(58, -12);
                    break;
            }
            sleep(50);
        }
        camera.webcam.stopStreaming();
        camera.webcam.stopRecordingPipeline();

        Trajectory park =
                drive.trajectoryBuilder(highPole.end())
                        .lineToLinearHeading(new Pose2d(parkPosition, Math.toRadians(90)))
                        .build();

        currentState = State.HIGH_POLE;

        waitForStart();

        while (opModeIsActive()) {
            switch (currentState) {

                case HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(highPole);
                        sleep(50);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(50);
                        belt.move(Consts.Belt.UP);
                        sleep(100);
                        lift.move(Consts.Lift.AUTO_LOW);
                        next(State.PARK);
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(park);

                        claw.reset();
                        belt.reset();
                        lift.reset();
                        turnTable.reset();
                        next(State.IDLE);
                    }
                    break;
            }
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("current state", currentState);
            telemetry.addData("busy", drive.isBusy());
            telemetry.addData("belt ", belt.getPosition());
            telemetry.addData("lift ", lift.getPosition());
            telemetry.update();
        }

    }
}