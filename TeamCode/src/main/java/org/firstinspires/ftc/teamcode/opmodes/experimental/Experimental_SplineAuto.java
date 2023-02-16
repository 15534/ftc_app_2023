package org.firstinspires.ftc.teamcode.opmodes.experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "SplineAutoEx")
@Config
@Disabled

public class Experimental_SplineAuto extends LinearOpMode {

    // facing field side
    //    Pose2d startingPos = new Pose2d(52, -12, Math.toRadians(0));
    int[] liftPosition = {275, 200, 130, 70, 0};

    State currentState = State.IDLE;
    double clawPosition, beltPosition;
    //    SampleMecanumDrive drive;
    ElapsedTime runtime = new ElapsedTime();

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    SampleMecanumDrive drive;
    TurnTable turntable = new TurnTable();
    Camera camera = new Camera();

    Pose2d coneStackStartPos;

    int conesCycled = 0;
    int numLow = 3;
    int numGround = 1;

    void next(State s) {
        double time = runtime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // new Pose2d(56, -13.75, Math.toRadians(90));

        Pose2d startingPos = new Pose2d(40, -62, Math.toRadians(90));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //        drive = new SampleMecanumDrive(hardwareMap);
        //        drive.setPoseEstimate(new Pose2d(52, -12, Math.toRadians(0))); // starting Pose2d

        // define trajectories
//        Trajectory FIRST_HIGH_POLE =
//                drive.trajectoryBuilder(startingPos)
//                        // moves to the first high junction (35.6, 0)
//                        // turns turntable 90 deg (counter-clockwise)
//                        // lifts to high
//                        .splineToSplineHeading(new Pose2d(36, -48, Math.toRadians(90)), Math.toRadians(90))
//                        .addDisplacementMarker(
//                                1,
//                                () -> {
//                                    turntable.move(90);
//                                })
//                        .addDisplacementMarker(
//                                10,
//                                () -> {
//                                    lift.move(Consts.Lift.HIGH);
//                                })
//                        .build();

//        Trajectory SPLINE_TO_CONESTACK = drive.trajectoryBuilder(new Pose2d(STRAFE.end().getX(), STRAFE.end().getY(), Math.toRadians(90)), true) // is heading math.toRadians(90)?
//                .splineTo(new Vector2d(36, -5), Math.toRadians(-90))
//                .addDisplacementMarker(7, () -> {
//                    turntable.move(-180);
//                })
//                .splineTo(new Vector2d(56,-12.1), Math.toRadians(0))
//                .build();

        Trajectory SPLINE_TO_HIGH = drive.trajectoryBuilder(startingPos)
                .addDisplacementMarker(
                        () -> {
                            lift.move(Consts.Lift.MEDIUM);
                        })
                .splineToConstantHeading(new Vector2d(36, -56), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(36, -24, Math.toRadians(90)), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(36, -20),
                        () -> {
                            belt.move(Consts.Belt.DOWN);
                        })
                .addSpatialMarker(new Vector2d(36, -16),
                        () -> {
                            lift.move(Consts.Lift.HIGH);
                        })
                .splineToSplineHeading(new Pose2d(32, -8, Math.toRadians(135)), Math.toRadians(90))
                .build();

        Trajectory SPLINE_TO_CONESTACK =
                drive.trajectoryBuilder(SPLINE_TO_HIGH.end())
                        .lineToSplineHeading(
                                new Pose2d(40, -13, Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            belt.move(Consts.Belt.UP);
                            sleep(100);
                            turntable.move(185);
                            lift.move(Consts.Lift.ZERO);
                        })
                        .build();

        Trajectory LINE_TO_CONESTACK =
                drive.trajectoryBuilder(SPLINE_TO_CONESTACK.end())
                        .lineTo(new Vector2d(57, -12.5))
                        .addDisplacementMarker(() -> {
                            belt.move(Consts.Belt.DOWN);
                            sleep(100);
                            lift.move(liftPosition[0]);
                        })
                        .build();

        Trajectory BACK_TO_GROUND = drive.trajectoryBuilder(SPLINE_TO_CONESTACK.end()).forward(9).build();
        Trajectory STRAFE_TO_GROUND = drive.trajectoryBuilder(BACK_TO_GROUND.end()).strafeRight(2).build();

        runtime.reset();

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        Vector2d parkPosition = new Vector2d();

        int position = -1;
//        while (!opModeIsActive() && !isStopRequested()) {
        position = camera.getPosition();
        telemetry.addData("position", position);
        telemetry.update();
//            sleep(50);
//        }

        waitForStart();

        position = camera.getPosition();
        telemetry.addData("position", position);
        telemetry.update();

        if (position == 1) {
            parkPosition = new Vector2d(12, -11.6);
        } else if (position == 2) {
            parkPosition = new Vector2d(36, -11.6);
        } else if (position == 3) {
            parkPosition = new Vector2d(58, -11.6);
        }

        // define park trajectory here because the value will be diff based off opencv values
        Trajectory PARK =
                drive.trajectoryBuilder(STRAFE_TO_GROUND.end()).lineTo(parkPosition).build();

//        telemetry.addData("streamStatus", "before");
//        telemetry.update();

        camera.webcam.stopStreaming();
        camera.webcam.stopRecordingPipeline();

//        telemetry.addData("streamStatus", "after");
//        telemetry.update();

        // state machine time!
        currentState = State.FIRST_HIGH_POLE;

        while (opModeIsActive()) {

            // only have to check current position/is not busy for subsystems moving in previous
            // states. if issues, add turntable checks.

            switch (currentState) { // input to switch
                case FIRST_HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(SPLINE_TO_HIGH);
                        next(State.DROP_FIRST_CONE);
                    }
                    break;

//                case STRAFE:
//                    drive.update();
//                    drive.updatePoseEstimate();
//                    if (!drive.isBusy()) {
//                        drive.followTrajectoryAsync(STRAFE);
//                        next(State.DROP_FIRST_CONE);
//                    }
//                    break;

                case DROP_FIRST_CONE:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
//                        belt.move(Consts.Belt.CONE_DROP);
//                        sleep(600);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(150);
                        next(State.SPLINE_TO_CONESTACK);
                    }
                    break;

                case SPLINE_TO_CONESTACK:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        drive.followTrajectory(SPLINE_TO_CONESTACK);
                        next(State.LINE_TO_CONESTACK);
                    }
                    break;

                case LINE_TO_CONESTACK:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        drive.followTrajectory(LINE_TO_CONESTACK);
                        next(State.REMOVE_FROM_CONESTACK);
                    }
                    break;

                case CYCLE_PICK_FROM_CONESTACK:
                    // shouldn't need to check lift here
                    // make a proper condition or smth
                    // claw.move(Consts.Claw.OPENCLAW);
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        sleep(300);
                        lift.move(liftPosition[conesCycled]);
                        belt.move(Consts.Belt.DOWN);
                        while (lift.right.isBusy() || lift.left.isBusy()) {
                            lift.getPosition();
                        }
                        next(State.REMOVE_FROM_CONESTACK);
                    }
                    break;

                case REMOVE_FROM_CONESTACK:
                    sleep(300);
                    claw.move(Consts.Claw.CLOSECLAW);
                    sleep(300); // to tighten around cone
                    lift.move(Consts.Lift.AUTO_LOW);
                    while (lift.right.isBusy() || lift.left.isBusy()) {
                        lift.getPosition(); // filler code just to wait out stuff, maybe
                        // swap for tele logging
                    }
                    sleep(300);
                    next(State.TURNTABLE_TO_SCORE);
                    break;

                case TURNTABLE_TO_SCORE:
                    if (conesCycled == numLow) {
                        turntable.move(-90);
                    } else {
                        turntable.move(48);
                    }
                    // bc auto ground could hit conestack
                    // tuned lol
                    while (turntable.motor.isBusy()) {
                        turntable.getPosition(); // filler code just to wait out stuff, maybe swap
                        // for tele logging
                    }
                    sleep(500);
                    if (conesCycled == numLow) {
                        // ground junction case
                        lift.move(Consts.Lift.AUTO_GROUND);
                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(BACK_TO_GROUND);
                            next(State.MOVE_TO_GROUND);
                        }
                    }
                    else {
                        next(State.SCORE);
                    }
                    break;

                case SCORE:
                    // lift is done moving. already confirmed claw in right place
                    if (!lift.left.isBusy() && !lift.right.isBusy()) {
                        sleep(500);
                        claw.move(Consts.Claw.OPENCLAW);
                        conesCycled++;
                        sleep(250); // sometimes claw doesn't open though it should
                        lift.move(Consts.Lift.AUTO_LOW);
                        while(lift.right.isBusy() || lift.left.isBusy()){
                            lift.getPosition();
                        }
                        if (conesCycled != numLow + numGround) {
                            // conesCycled = 4 when we score the first 4 cones
                            // kinda scuffed sol tbh
                            next(State.RESET);
                        } else {
                            turntable.move(0);
                            drive.followTrajectoryAsync(PARK);
                            next(State.PARK);
                        }
                    }
                    break;
//
////                case MOVE_TO_GROUND:
////                    drive.update();
////                    drive.updatePoseEstimate();
////                    if (!drive.isBusy()) {
////                        drive.followTrajectoryAsync(STRAFE_TO_GROUND);
////                        next(State.STRAFE_TO_GROUND);
////                    }
////                    break;
//
//                case STRAFE_TO_GROUND:
//                    drive.update();
//                    drive.updatePoseEstimate();
//                    if (!drive.isBusy()) {
//                        next(State.SCORE);
//                    }
//                    break;
//
//                case RESET:
//                    // checking claw in right position, because only one that moved in previous
//                    // state
//                    //                    if (clawPosition == Consts.CLAW_OPEN_LIMIT) {
//                    turntable.move(-180);
//                    while (turntable.motor.isBusy()) {
//                        turntable.getPosition();
//                    }
//                    sleep(200);
////                    claw.move(Consts.Claw.CLOSECLAW);
//                    //                        belt.move(Consts.Belt.UP);
//                    next(State.PICK_FROM_CONESTACK);
//                    //                    }
//                    break;
//
//                case PARK:
//                    claw.move(Consts.Claw.CLOSECLAW);
//                    belt.move(Consts.Belt.UP);
//                    lift.move(Consts.Lift.ZERO);
//
//                    drive.update();
//                    drive.updatePoseEstimate();
//                    if (!drive.isBusy()) {
//                        next(State.IDLE);
//                    }
//                    break;
            }

            //drive.update();

            telemetry.addData("current state", currentState);
            telemetry.addData("lift position", lift.getPosition());

            telemetry.addData("formatted belt position", beltPosition);
            telemetry.addData("belt position", belt.getPosition());

            telemetry.addData("formatted claw position", clawPosition);
            telemetry.addData("claw position", claw.getPosition());

            telemetry.addData("belt condition", beltPosition == Consts.BELT_UP_LIMIT);
            telemetry.addData("claw condition", clawPosition == Consts.CLAW_CLOSE_LIMIT);

            telemetry.addData("open cv position", position);
            telemetry.addData("park pos", String.valueOf((parkPosition.getX())), parkPosition.getY());

            telemetry.update();
        }
    }

    // why define here? not earlier?

    enum State {
        PARK,
        RESET,
        FIRST_HIGH_POLE,
        STRAFE,
        DROP_FIRST_CONE,
        PREPARE_TO_TURN,
        TURN_TO_CONESTACK,
        GO_TOWARDS_CONESTACK,
        CYCLE_PICK_FROM_CONESTACK,
        REMOVE_FROM_CONESTACK,
        TURNTABLE_TO_SCORE,
        SCORE,
        STRAFE_TO_GROUND,
        MOVE_TO_GROUND,
        IDLE,
        SPLINE_TO_CONESTACK,
        LINE_TO_CONESTACK
    }
}
