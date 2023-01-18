package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "RedAutoV2")
@Config
public class RedAutoV2 extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    Pose2d startingPos = new Pose2d(36, -62, Math.toRadians(90));
    ElapsedTime runtime = new ElapsedTime();
    int cyclesCompleted = 0;
    int[] liftPosition = {80, 60, 40, 27, 0};

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    TurnTable turntable = new TurnTable();

    Trajectory firstLowPole, firstMedPole, firstConeStack, coneStack, placeHighPole, park;
    Trajectory currentTrajectory;
    TrajectorySequence turnAfterHighPole;
    TrajectorySequence currentTrajectorySequence;




    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public void buildTrajectories() {



        firstLowPole =
                drive.trajectoryBuilder(startingPos)
                        .addSpatialMarker(
                                new Vector2d(36, -62),
                                () -> {
                                    lift.moveLift(Constants.LiftTargets.PICKUP);
                                    belt.moveBelt(Constants.IntakeTargets.PICKUP);
                                    claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                                })
                        .addDisplacementMarker(
                                () -> {
                                    turntable.turn(0);
                                })
                        .lineTo(new Vector2d(36, -40))
                        .build();

        firstMedPole =
                drive.trajectoryBuilder(firstLowPole.end())
                        .lineTo(new Vector2d(12, -12))
                        .addDisplacementMarker(
                                2,
                                () -> {
                                    turntable.turn(0);
                                    lift.moveLift(Constants.LiftTargets.PICKUP);
                                })
                        .build();

        turnAfterHighPole =
                drive.trajectorySequenceBuilder(firstMedPole.end())
                        .turn(Math.toRadians(-90))
                        .build();

        firstConeStack =
                drive.trajectoryBuilder(turnAfterHighPole.end())
                        .addDisplacementMarker(
                                0,
                                () -> {
                                    belt.moveBelt(Constants.IntakeTargets.DOWN);
                                    lift.moveLift(liftPosition[(cyclesCompleted)]);
                                    claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                                    turntable.turn(0);
                                })
                        .lineTo(new Vector2d(54, -12))
                        .addDisplacementMarker(() ->{
                            claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                            sleep(200);
                            lift.moveLift(Constants.LiftTargets.HIGH);
                            sleep(200);
                        })

                        .build();

        placeHighPole =
                drive.trajectoryBuilder(new Pose2d(54, -12))
                        .lineTo(new Vector2d(24, -12))
                        .addDisplacementMarker(
                                3,
                                () -> {
//                                    belt.moveBelt(Constants.IntakeTargets.PICKUP);
                                    lift.moveLift(Constants.LiftTargets.HIGH);
                                    turntable.turn(90);
                                })
                        .build();

        coneStack =
                drive.trajectoryBuilder(new Pose2d(24, -14))
                        .addDisplacementMarker(
                                0,
                                () -> {
                                    belt.moveBelt(Constants.IntakeTargets.DOWN);
                                    lift.moveLift(liftPosition[(cyclesCompleted)]);
                                    claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                                    turntable.turn(0);
                                })
                        .lineTo(new Vector2d(54, -12))
                        .addDisplacementMarker(() ->{
                            claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                            sleep(200);
                            lift.moveLift(Constants.LiftTargets.HIGH);
                            sleep(200);
                        })

                        .build();

        park =
                drive.trajectoryBuilder(placeHighPole.end())
                        .splineToConstantHeading(new Vector2d(12, -30), Math.toRadians(90))
                        .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));

        buildTrajectories();

        runtime.reset();
        waitForStart();

        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        currentState = State.PRELOAD_SCORE_LOW;

        while (opModeIsActive()) {
            telemetry.addLine("running");

            // @TODO: Determine cycle-time using "elapsed" variable
            //            double elapsed = runtime.seconds() - time;

            switch (currentState) {
                case PRELOAD_SCORE_LOW:
                    // (36, -62) --> (36,-40)move to first low pole
                    // lift low
                    // belt down
                    // open claw

                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstLowPole);
                        lift.moveLift(Constants.LiftTargets.LOW);
                        belt.moveBelt(Constants.IntakeTargets.DOWN);
                        sleep(400);
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);

                        next(State.IDLE);

                    }
                    break;

                ///////////////////////////////////////////////////////////////

                case SLEEVE_SCORE_MED:
                    if (!drive.isBusy()) {
                        sleep(275);

//                        belt.moveBelt(Constants.IntakeTargets.DROPOFF);
                        belt.moveBelt(Constants.IntakeTargets.DOWN);
                        sleep(400);
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);

                        sleep(200);
                        belt.moveBelt(Constants.IntakeTargets.PICKUP);
                        claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
//                        sleep(400);

                        sleep(4000);


                        next(State.FIRST_CONESTACK);
                    }
                    break;



                case FIRST_CONESTACK:
                    // move belt upwards
                    // move to
                    // (12, 12)
                    if (!drive.isBusy()) {

                        belt.moveBelt(Constants.IntakeTargets.PICKUP); // moves it up

                        drive.followTrajectoryAsync(firstMedPole);
                        next(State.TURN_AFTER_FIRST_SCORE); // change to turn after first score
                    }
                    break;
                case TURN_AFTER_FIRST_SCORE:
                    // turn 90 deg to face cone stack
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(turnAfterHighPole);
                        next(State.GO_HIGHJUNC_CONESTACKS);
                    }
                    break;
                case GO_HIGHJUNC_CONESTACKS:
                    // go to
                    // (60, -12)
                    // conestacks
                    if (!drive.isBusy()) {
//                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        if (cyclesCompleted == 0) {
                            drive.followTrajectoryAsync(firstConeStack);
                        }
                        else {
                            drive.followTrajectoryAsync(coneStack);
                        }
                        claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                        lift.moveLift(Constants.LiftTargets.LOW);
                        belt.moveBelt(Constants.IntakeTargets.HOLD);
                        next(State.PLACE_HIGHJUNC_CONE);
                    }
                    break;
                case PLACE_HIGHJUNC_CONE:
                    // go to
                    // (12, -12)
                    // high junction
                    // increment cycles count
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(placeHighPole);
                        belt.moveBelt(Constants.IntakeTargets.DOWN);

//                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        cyclesCompleted++;
                        if (cyclesCompleted == 6) {
                            next(State.PARK);
                        } else {
//                            claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                            next(State.GO_HIGHJUNC_CONESTACKS);
                        }
                    }
                    break;
                case PARK:
                    // parks to splineConstantHeading
                    // (12, -30, 90˚)
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(park);
                        next(State.IDLE);
                    }
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("current state", currentState);
            telemetry.addData("busy", drive.isBusy());
            telemetry.addData("cycles ", cyclesCompleted);
            telemetry.update();
        }
    }
    // TODO: 1/14/2023
    // get the translational pids tuned

    // For drivetrain states/trajectories, GO_{FIRST PLACE}_{LAST PLACE}
    enum State {
        PRELOAD_SCORE_LOW,
        SLEEVE_SCORE_MED,
        FIRST_CONESTACK,
        TURN_AFTER_FIRST_SCORE,
        GO_HIGHJUNC_CONESTACKS,
        PLACE_HIGHJUNC_CONE,
        PARK,
        IDLE
    }
}
