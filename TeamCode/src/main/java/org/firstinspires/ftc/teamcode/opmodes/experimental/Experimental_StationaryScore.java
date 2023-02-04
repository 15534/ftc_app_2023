package org.firstinspires.ftc.teamcode.opmodes.experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "StationaryEx")
@Config
public class Experimental_StationaryScore extends LinearOpMode {

    // facing field side
    //    Pose2d startingPos = new Pose2d(52, -12, Math.toRadians(0));
    int[] liftPosition = {290, 200, 130, 70, 0};

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

    int conesCycled = 0;

    void next(State s) {
        double time = runtime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startingPos = new Pose2d(56.4, -13.30, Math.toRadians(90));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //        drive = new SampleMecanumDrive(hardwareMap);
        //        drive.setPoseEstimate(new Pose2d(52, -12, Math.toRadians(0))); // starting Pose2d

        // define trajectories

        Trajectory BACK_TO_GROUND = drive.trajectoryBuilder(startingPos).back(9).build();
        Trajectory STRAFE_TO_GROUND = drive.trajectoryBuilder(BACK_TO_GROUND.end()).strafeLeft(2).build();

        runtime.reset();

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        Vector2d parkPosition = new Vector2d();

        int position = -1;
        while (!opModeIsActive() && !isStopRequested()) {
            position = camera.getPosition();
            telemetry.addData("position", position);

            telemetry.update();
            sleep(50);
        }

        if (position == 1) {
            parkPosition = new Vector2d(12, -11.6);
        } else if (position == 2) {
            parkPosition = new Vector2d(36, -11.6);
        } else if (position == 3) {
            parkPosition = new Vector2d(58, -11.6);
        }

        // define park trajectory here because the value will be diff based off opencv values
//        Trajectory PARK =
//                drive.trajectoryBuilder(CYCLE_HIGH_POLE.end()).lineTo(parkPosition).build();

        waitForStart();



        telemetry.addData("READY", "");
        telemetry.update();

        // state machine time!
        currentState = State.PICK_FROM_CONESTACK;

        while (opModeIsActive()) {

            // only have to check current position/is not busy for subsystems moving in previous
            // states. if issues, add turntable checks.

            switch (currentState) { // input to switch
                case PICK_FROM_CONESTACK:
                    // shouldn't need to check lift here
                    // make a proper condition or smth
                    claw.move(Consts.Claw.OPENCLAW);
                    sleep(200);
                    lift.move(liftPosition[conesCycled]);
                    belt.move(Consts.Belt.DOWN);
                    while(lift.right.isBusy() || lift.left.isBusy()){
                        lift.getPosition();
                    }
                    next(State.REMOVE_FROM_CONESTACK);
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
                    if (conesCycled == 4) {
                        turntable.move(90);
                    } else {
                        turntable.move(-124);
                    }
                    // tuned lol
                    while (turntable.motor.isBusy()) {
                        turntable.getPosition(); // filler code just to wait out stuff, maybe swap
                        // for tele logging
                    }
                    sleep(700);
                    if (conesCycled != 4) {
//                        sleep(300);
                        next(State.SCORE);
                    }
                    else {
                        // ground junction case
                        lift.move(Consts.Lift.AUTO_GROUND);
                        drive.followTrajectoryAsync(BACK_TO_GROUND);
                        next(State.MOVE_TO_GROUND);
                    }

                    break;

                case SCORE:
                    // lift is done moving. already confirmed claw in right place
                    if (!lift.left.isBusy() && !lift.right.isBusy()) {
                        claw.move(Consts.Claw.OPENCLAW);
                        conesCycled++;
                        sleep(250); // sometimes claw doesn't open though it should
                        lift.move(Consts.Lift.AUTO_LOW);
                        while(lift.right.isBusy() || lift.left.isBusy()){
                            lift.getPosition();
                        }
                        if (conesCycled != 5) {
                            // conesCycled = 4 when we score the first 4 cones
                            // kinda scuffed sol tbh
                            next(State.RESET);
                        } else {
                            turntable.move(0);
                            while(turntable.motor.isBusy()){
                                turntable.getPosition();
                            }
                            next(State.IDLE);
                        }
                    }
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
            }

            drive.update();

            telemetry.addData("current state", currentState);
            telemetry.addData("lift position", lift.getPosition());

            telemetry.addData("formatted belt position", beltPosition);
            telemetry.addData("belt position", belt.getPosition());

            telemetry.addData("formatted claw position", clawPosition);
            telemetry.addData("claw position", claw.getPosition());

            telemetry.addData("belt condition", beltPosition == Consts.BELT_UP_LIMIT);
            telemetry.addData("claw condition", clawPosition == Consts.CLAW_CLOSE_LIMIT);

            telemetry.update();
        }
    }

    // why define here? not earlier?

    enum State {
        RESET,
        REMOVE_FROM_CONESTACK,
        PICK_FROM_CONESTACK,
        TURNTABLE_TO_SCORE,
        SCORE,
        STRAFE_TO_GROUND,
        MOVE_TO_GROUND,
        IDLE
    }
}
