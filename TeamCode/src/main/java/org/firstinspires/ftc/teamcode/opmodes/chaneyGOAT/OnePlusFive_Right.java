package org.firstinspires.ftc.teamcode.opmodes.chaneyGOAT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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

@Autonomous(name = "1+5_RIGHT", group = "ChaneyTheGOAT")
@Config

public class OnePlusFive_Right extends LinearOpMode {

    enum State {
        HIGH_POLE,
        TO_CONE_STACK,
        LOW_JUNCTION_SCORE,
        TO_GROUND_JUNCTION,
        BACK_TO_CONE,
        TO_MEDIUM_JUNCTION,
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
    Vector2d mediumJunction = new Vector2d(24.2, -12);
    Vector2d parkPosition = new Vector2d();

    int[] liftPosition = {245, 160, 100, 35, 0};
    int lowJunctionCycles = 3;
    int targetLiftPosition = 0;

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
        TrajectorySequence toConeStack =
            drive.trajectorySequenceBuilder(highPole.end())
                    .splineToLinearHeading(new Pose2d(34.47, -13.02, Math.toRadians(180)), Math.toRadians(-9.46))
                    .lineToSplineHeading(new Pose2d(56.2, -14.1, Math.toRadians(180)))

                            .addSpatialMarker(new Vector2d(48, -12),
                                    () -> {
                                        lift.move(liftPosition[0]);
                                    })
                            .addSpatialMarker(new Vector2d(30, -12),
                                    () -> {
                                        turnTable.move(185);
                                    })
                            .build();
        Trajectory faceLowJunction =
                drive.trajectoryBuilder(toConeStack.end())
                        .back(0.1)
                        .addSpatialMarker(new Vector2d(56, -12),
                                () -> {
                                    turnTable.move(58);
                                })
                        .addSpatialMarker(new Vector2d(56, -12),
                                () -> {
                                    lift.move(Consts.Lift.AUTO_LOW);
                                })
                        .build();

        Trajectory faceConeStack =
                drive.trajectoryBuilder(toConeStack.end())
                        .back(0.1)
                        .addSpatialMarker(new Vector2d(56, -12),
                                () -> {
                                    turnTable.move(185);
                                })
                        .build();

        Trajectory toGroundJunction =
                drive.trajectoryBuilder(toConeStack.end())
                        .lineTo(new Vector2d(54.70, -10.70))
                        .addSpatialMarker(new Vector2d(55, -12),
                                () -> {
                                    turnTable.move(-52);
                                })
                        .addSpatialMarker(new Vector2d(55, -12),
                                () -> {
                                    lift.move(Consts.Lift.LOW);
                                })
                        .build();
        Trajectory toMediumJunction =
                drive.trajectoryBuilder(toConeStack.end())
                        .lineTo(mediumJunction)
                        .addSpatialMarker(new Vector2d(56, -12.4),
                                () -> {
                                    lift.move(Consts.Lift.AUTO_MEDIUM);
                                })
                        .addSpatialMarker(new Vector2d(56, -12),
                                () -> {
                                    turnTable.move(90);
                                })
                        .build();

        Trajectory toConeStackAfterGroundJunction =
                drive.trajectoryBuilder(toGroundJunction.end())
                        .lineTo(new Vector2d(55.80, -13.8))
                        .addSpatialMarker(new Vector2d(40, -12),
                                () -> {
                                    turnTable.move(185);
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
                drive.trajectoryBuilder(toMediumJunction.end())
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
                        next(State.TO_CONE_STACK);
                    }
                    break;
                case TO_CONE_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(toConeStack);
                        belt.moveAbsolute(Consts.BELT_AUTO_DOWN_LIMIT);
                        sleep(150);
                        next(State.LOW_JUNCTION_SCORE);
                    }
                    break;
                case LOW_JUNCTION_SCORE:
                    if (!drive.isBusy()) {

                        int conesCycled = 0;

                        claw.move(Consts.Claw.CLOSECLAW);
                        sleep(250);
                        lift.move(liftPosition[targetLiftPosition]+255);

                        while (conesCycled < lowJunctionCycles){

                            sleep(250);
                            drive.followTrajectory(faceLowJunction);
                            sleep(850);

                            claw.move(Consts.Claw.OPENCLAW);
                            sleep(300);

                            conesCycled+=1;
                            targetLiftPosition+=1;

                            drive.followTrajectory(faceConeStack);
                            sleep(400);
                            lift.move(liftPosition[targetLiftPosition]);
                            sleep(400);

                            claw.move(Consts.Claw.CLOSECLAW);
                            sleep(300);

                            lift.move(liftPosition[targetLiftPosition]+345);
                            sleep(300);
                        }

                        next(State.TO_GROUND_JUNCTION);
                    }
                    break;

                case TO_GROUND_JUNCTION:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(toGroundJunction);
                        sleep(700);
                        lift.move(Consts.Lift.AUTO_GROUND);

                        sleep(700);
                        claw.move(Consts.Claw.OPENCLAW);

                        sleep(700);
                        lift.move(Consts.Lift.AUTO_LOW);
                        sleep(700);

                        next(State.BACK_TO_CONE);
                    }
                    break;
                case BACK_TO_CONE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(toConeStackAfterGroundJunction);

                        sleep(600);
                        lift.move(liftPosition[4]);
                        sleep(600);
//                        belt.move(Consts.Belt.CONE_DROP);
                        belt.moveAbsolute(Consts.BELT_AUTO_DOWN_LIMIT);
                        sleep(600);
                        claw.move(Consts.Claw.CLOSECLAW);
                        sleep(600);
                        lift.move(Consts.Lift.AUTO_LOW);

                        next(State.TO_MEDIUM_JUNCTION);
                    }
                    break;
                case TO_MEDIUM_JUNCTION:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(toMediumJunction);
                        sleep(600);

                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(50);

                        belt.move(Consts.Belt.UP);
                        sleep(50);

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
            telemetry.addData("cycles ", lowJunctionCycles);
            telemetry.addData("belt ", belt.getPosition());
            telemetry.addData("lift ", lift.getPosition());
            telemetry.update();
        }

    }
}