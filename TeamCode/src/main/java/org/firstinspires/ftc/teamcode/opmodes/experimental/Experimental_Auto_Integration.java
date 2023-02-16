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

@Autonomous(name = "Integrated_Auto")
@Config
@Disabled

public class Experimental_Auto_Integration extends LinearOpMode {

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
    int numLow = 2;
    int numGround = 1;

    void next(State s) {
        double time = runtime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // new Pose2d(56, -13.75, Math.toRadians(90));

        Pose2d startingPos = new Pose2d(39.5, -62, Math.toRadians(90));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //        drive = new SampleMecanumDrive(hardwareMap);
        //        drive.setPoseEstimate(new Pose2d(52, -12, Math.toRadians(0))); // starting Pose2d

        // define trajectories
        Trajectory STRAFE_FIRST_HIGH_POLE = drive.trajectoryBuilder(startingPos)
                .strafeLeft(3.5)
//                .lineTo(new Vector2d(36, -62))
                .build();

        Trajectory FIRST_HIGH_POLE =
                drive.trajectoryBuilder(STRAFE_FIRST_HIGH_POLE.end())
                        // moves to the first high junction (35.6, 0)
                        // turns turntable 90 deg (counter-clockwise)
                        // lifts to high
                        .lineTo(new Vector2d(36, 0))
                        .addDisplacementMarker(
                                1,
                                () -> {
                                    turntable.move(90);
                                })
                        .addDisplacementMarker(
                                10,
                                () -> {
                                    lift.move(Consts.Lift.AUTO_HIGH);
                                })
                        .build();

        Trajectory STRAFE = drive.trajectoryBuilder(FIRST_HIGH_POLE.end()).strafeLeft(0.2).build();

        // @TODO: combine PREPARE_TO_TURN and the turning
        Trajectory PREPARE_TO_TURN =
                drive.trajectoryBuilder(STRAFE.end()).lineTo(new Vector2d(36, -12)).build();

        coneStackStartPos =
                new Pose2d(PREPARE_TO_TURN.end().getX(), PREPARE_TO_TURN.end().getY(), 0);

        Trajectory GO_TOWARDS_CONESTACK =
                drive.trajectoryBuilder(coneStackStartPos)
                        // move to conestacks, can be tuned for more accuracy for pickup
                        // 56, -13.75
                        .lineTo(new Vector2d(55.5, -12.1))
                        .build();

        Trajectory BACK_TO_GROUND = drive.trajectoryBuilder(GO_TOWARDS_CONESTACK.end()).back(9).build();
        Trajectory STRAFE_TO_GROUND = drive.trajectoryBuilder(BACK_TO_GROUND.end()).strafeLeft(1).build();

        runtime.reset();

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        Vector2d parkPosition = new Vector2d();

        int position = -1;

        position = camera.getPosition();
        telemetry.addData("position", position);

        telemetry.update();
        sleep(50);

        waitForStart();

        position = camera.getPosition();
        telemetry.addData("position", position);
        sleep(50);

        if (position == 1) {
            parkPosition = new Vector2d(12, -12.3);
            telemetry.update();
        } else if (position == 2) {
            parkPosition = new Vector2d(36, -12.3);
            telemetry.update();
        } else if (position == 3) {
            parkPosition = new Vector2d(58, -12.3);
            telemetry.update();
        } else if (position == -1) {
            parkPosition = new Vector2d(12, -12);
            telemetry.update();
        }

        // define park trajectory here because the value will be diff based off opencv values
        Trajectory PARK =
                drive.trajectoryBuilder(STRAFE_TO_GROUND.end()).lineTo(parkPosition).build();

        telemetry.addData("READY", "");
        telemetry.update();

        camera.webcam.stopStreaming();
        camera.webcam.stopRecordingPipeline();

        // state machine time!
        currentState = State.FIRST_HIGH_POLE;

        while (opModeIsActive()) {

            // only have to check current position/is not busy for subsystems moving in previous
            // states. if issues, add turntable checks.

            switch (currentState) { // input to switch
                case STRAFE_FIRST_HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(STRAFE_FIRST_HIGH_POLE);
                        next(State.FIRST_HIGH_POLE);
                    }
                case FIRST_HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(FIRST_HIGH_POLE);
                        next(State.STRAFE);
                    }
                    break;

                case STRAFE:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(STRAFE);
                        next(State.DROP_FIRST_CONE);
                    }
                    break;

                case DROP_FIRST_CONE:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        belt.move(Consts.Belt.CONE_DROP);
                        sleep(600);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(150);
                        next(State.PREPARE_TO_TURN);
                    }
                    break;

                case PREPARE_TO_TURN:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        sleep(500);
                        belt.move(Consts.Belt.UP);
                        lift.move(Consts.Lift.ZERO);
                        turntable.move(0);

                        drive.followTrajectoryAsync(PREPARE_TO_TURN);

                        next(State.TURN_TO_CONESTACK);
                    }
                    break;

                case TURN_TO_CONESTACK:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        drive.turn(Math.toRadians(-90));
                        next(State.GO_TOWARDS_CONESTACK);
                    }
                    break;

                case GO_TOWARDS_CONESTACK:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        sleep(750);
                        // to pick up first. will move lift as well here later.
                        turntable.move(5);
                        // lift.move(liftPosition[cyclesCompleted]);
                        // belt.move(Consts.Belt.DOWN);
                        drive.followTrajectoryAsync(GO_TOWARDS_CONESTACK);
                        next(State.PICK_FROM_CONESTACK);
                    }
                    break;

                case PICK_FROM_CONESTACK:
                    // shouldn't need to check lift here
                    // make a proper condition or smth
                    // claw.move(Consts.Claw.OPENCLAW);
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        sleep(300);
                        lift.move(liftPosition[conesCycled]);
                        belt.move(Consts.Belt.CONE_DROP);
                        while(Math.abs(lift.getPosition() - lift.getTarget()) > 20){
                            telemetry.addData("current", lift.getPosition());
                            telemetry.addData("target", lift.getTarget());
                            telemetry.update();
                        }
                        telemetry.addData("done lift", "yes");
                        telemetry.update();
                        next(State.REMOVE_FROM_CONESTACK);
                    }
                    break;

                case REMOVE_FROM_CONESTACK:
                    sleep(300);
                    claw.move(Consts.Claw.CLOSECLAW);
                    sleep(300); // to tighten around cone
                    lift.move(Consts.Lift.AUTO_LOW);
                    while(Math.abs(lift.getPosition() - lift.getTarget()) > 20){
                        telemetry.addData("current", lift.getPosition());
                        telemetry.addData("target", lift.getTarget());
                        telemetry.update();
                    }
                    telemetry.addData("done lift", "yes");
                    telemetry.update();
                    sleep(300);
                    next(State.TURNTABLE_TO_SCORE);
                    break;

                case TURNTABLE_TO_SCORE:
                    if (conesCycled == numLow) {
                        turntable.move(90);
                    } else {
                        turntable.move(-124);
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
//                    if (!lift.left.isBusy() && !lift.right.isBusy()) {
                        sleep(500);
                        claw.move(Consts.Claw.OPENCLAW);
                        conesCycled++;
                        sleep(250); // sometimes claw doesn't open though it should
//                        lift.move(Consts.Lift.AUTO_LOW);
                        telemetry.addData("done belt", "no");
                        telemetry.update();
//                        while(Math.abs(lift.getPosition() - lift.getTarget()) > 20){
//                            telemetry.addData("current", lift.getPosition());
//                            telemetry.addData("target", lift.getTarget());
//                            telemetry.update();
//                        }
                        belt.move(Consts.Belt.UP);
                        telemetry.addData("done belt", "yes");
                        telemetry.update();
                        if (conesCycled != numLow + numGround) {
                            // conesCycled = 4 when we score the first 4 cones
                            // kinda scuffed sol tbh

                            next(State.RESET);
                        } else {
//                            belt.move(Consts.Belt.UP);
                            turntable.move(0);
                            lift.move(0);
                            claw.move(Consts.Claw.CLOSECLAW);
                            drive.followTrajectoryAsync(PARK);
                            next(State.PARK);
                        }
//                    }
                    break;

                case MOVE_TO_GROUND:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(STRAFE_TO_GROUND);
                        next(State.STRAFE_TO_GROUND);
                    }
                    break;

                case STRAFE_TO_GROUND:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        next(State.SCORE);
                    }
                    break;

                case RESET:
                    // checking claw in right position, because only one that moved in previous
                    // state
                    //                    if (clawPosition == Consts.CLAW_OPEN_LIMIT) {
                    turntable.move(5);
                    while (turntable.motor.isBusy()) {
                        turntable.getPosition();
                    }
                    sleep(200);
//                    claw.move(Consts.Claw.CLOSECLAW);
                    //                        belt.move(Consts.Belt.UP);
                    next(State.PICK_FROM_CONESTACK);
                    //                    }
                    break;

                case PARK:
                    drive.update();
                    drive.updatePoseEstimate();
                    if (!drive.isBusy()) {
                        claw.reset();
                        belt.reset();
                        lift.reset();
                        next(State.IDLE);
                    }
                    break;
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
        STRAFE_FIRST_HIGH_POLE,
        FIRST_HIGH_POLE,
        STRAFE,
        DROP_FIRST_CONE,
        PREPARE_TO_TURN,
        TURN_TO_CONESTACK,
        GO_TOWARDS_CONESTACK,
        PICK_FROM_CONESTACK,
        REMOVE_FROM_CONESTACK,
        TURNTABLE_TO_SCORE,
        SCORE,
        STRAFE_TO_GROUND,
        MOVE_TO_GROUND,
        IDLE
    }
}
