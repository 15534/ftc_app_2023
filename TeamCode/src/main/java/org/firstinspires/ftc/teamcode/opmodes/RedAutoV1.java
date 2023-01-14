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
import org.firstinspires.ftc.teamcode.subsystems.OldBelt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "RedAutoV1")
@Config
public class RedAutoV1 extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    Pose2d startingPos = new Pose2d(12, -62, Math.toRadians(90));
    ElapsedTime runtime = new ElapsedTime();
    int cyclesCompleted = 0;

    Lift lift = new Lift();
    Claw claw = new Claw();
    OldBelt oldBelt = new OldBelt();
    TurnTable turntable = new TurnTable();

    Trajectory firstHighPole, firstConeStack, coneStack, placeHighPole, park;
    TrajectorySequence turnAfterHighPole;

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public void buildTrajectories() {

        /*
        IMPORTANT POINTS:
        (12, 24): Corner of cone
        (52, 12): Pickup position
        (20, 12): Corner of cone
        (10, 35): Test ending position
         */

        /*
        ORDER OF TRAJECTORIES:
        firstHighPole
        firstConeStack

        Loop for x amount of cycles {
            placeHighPole
            coneStack
        }

        Park
        */

        firstHighPole =
                drive.trajectoryBuilder(startingPos)
                        /*
                        .addSpatialMarker(new Vector2d(12, -46), () -> {
                            lift.moveLift(Constants.LiftTargets.HIGH);
                        })
                        .addDisplacementMarker(
                                () -> {
                                    turntable.turn(90);
                                })

                         */
                        .lineTo(new Vector2d(12, -24))
                        .build();

        firstConeStack =
                drive.trajectoryBuilder(firstHighPole.end())
                        .lineTo(new Vector2d(12, -12))
                        /*

                        .addDisplacementMarker(
                                2,
                                () -> {
                                    turntable.turn(0);
                                    lift.moveLift(Constants.LiftTargets.PICKUP);
                                })

                         */
                        .build();

        turnAfterHighPole =
                drive.trajectorySequenceBuilder(firstConeStack.end())
                        .turn(Math.toRadians(-90))
                        .build();

        coneStack =
                drive.trajectoryBuilder(turnAfterHighPole.end())
                        .lineTo(new Vector2d(60, -12))
                        .build();

        placeHighPole =
                drive.trajectoryBuilder(coneStack.end())
                        .lineTo(new Vector2d(12, -12))
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
        drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(90)));

        buildTrajectories();

        runtime.reset();
        waitForStart();

        claw.init(hardwareMap);
        oldBelt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        currentState = State.GO_SUBSTATION_HIGHJUNC;

        while (opModeIsActive()) {
            telemetry.addLine("running");

            // @TODO: Determine cycle-time using "elapsed" variable
            //            double elapsed = runtime.seconds() - time;

            switch (currentState) {
                case GO_SUBSTATION_HIGHJUNC:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstHighPole);
                        //                        cyclesCompleted++;
                        next(State.DROP_FIRST_CONE);
                    }
                    break;
                case DROP_FIRST_CONE:
                    if (!drive.isBusy()) {
                        sleep(275);

                        /*oldBelt.moveBelt(Constants.IntakeTargets.DROPOFF);

                        long t = System.currentTimeMillis();
                        long end = t + 1250;

                        while (System.currentTimeMillis() < end) {
                            oldBelt.updateBeltPosition();
                        }

                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        sleep(2000);

                         */
                        next(State.FIRST_CONESTACK);
                    }
                    break;
                case FIRST_CONESTACK:
                    if (!drive.isBusy()) {

                        /*
                        oldBelt.moveBelt(Constants.IntakeTargets.PICKUP); // moves it up

                        long t = System.currentTimeMillis();
                        long end = t + 1000;

                        while (System.currentTimeMillis() < end) {
                            oldBelt.updateBeltPosition();
                        }
                        */

                        drive.followTrajectoryAsync(firstConeStack);
                        next(State.TURN_AFTER_FIRST_SCORE); //change to turn after first score
                    }
                    break;
                case TURN_AFTER_FIRST_SCORE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(turnAfterHighPole);
                        next(State.GO_HIGHJUNC_CONESTACKS);
                    }
                    break;
                case GO_HIGHJUNC_CONESTACKS:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(coneStack);
                        next(State.PLACE_HIGHJUNC_CONE);
                    }
                    break;
                case PLACE_HIGHJUNC_CONE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(placeHighPole);
                        cyclesCompleted++;
                        if (cyclesCompleted == 6) {
                            next(State.PARK);
                        } else {
                            next(State.GO_HIGHJUNC_CONESTACKS);
                        }
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(park);
                        next(State.IDLE);
                    }
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            oldBelt.updateBeltPosition();

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
        GO_SUBSTATION_HIGHJUNC,
        DROP_FIRST_CONE,
        FIRST_CONESTACK,
        TURN_AFTER_FIRST_SCORE,
        GO_HIGHJUNC_CONESTACKS,
        PLACE_HIGHJUNC_CONE,
        GO_CONESTACKS_HIGHJUNC,
        PARK,
        IDLE
    }
}
