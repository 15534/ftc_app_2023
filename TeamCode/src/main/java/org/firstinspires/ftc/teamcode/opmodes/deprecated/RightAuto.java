package org.firstinspires.ftc.teamcode.opmodes.deprecated;

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

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;
import org.firstinspires.ftc.teamcode.utility.OpenCV;

@Autonomous(name = "RightAuto")
@Config
@Disabled

public class RightAuto extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    Pose2d startingPos = new Pose2d(36, -62, Math.toRadians(90));
    ElapsedTime runtime = new ElapsedTime();
    int cyclesCompleted = 0;
    int[] liftPosition = {290, 200, 130, 70, 0};

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    TurnTable turntable = new TurnTable();
    Camera camera = new Camera();

    Pose2d coneStackStartPos;

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));

        Trajectory FIRST_HIGH_POLE =
                drive.trajectoryBuilder(startingPos)
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
                                    lift.move(Consts.Lift.HIGH);
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
                        .lineTo(new Vector2d(56.5, -11.6))
                        .build();

        Trajectory CYCLE_HIGH_POLE =
                drive.trajectoryBuilder(GO_TOWARDS_CONESTACK.end())
                        // move back to high pole, slowed down drive constants
                        // lifts and turntable (counter-clockwise)
                        .lineTo(
                                new Vector2d(23, -12.4),
                                SampleMecanumDrive.getVelocityConstraint(
                                        15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(
                                        DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(
                                10,
                                () -> {
                                    turntable.move(90);
                                    lift.move(1850);
                                })
                        .build();

        Trajectory CYCLE_CONESTACK =
                drive.trajectoryBuilder(CYCLE_HIGH_POLE.end())
                        // go back to conestacks after placing at high pole for CYCLING
                        .lineTo(new Vector2d(56, -11.6))
                        .build();

        runtime.reset();

        Vector2d parkPosition = new Vector2d();

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

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
        Trajectory PARK =
                drive.trajectoryBuilder(CYCLE_HIGH_POLE.end()).lineTo(parkPosition).build();

        waitForStart();

        telemetry.addData("READY", "");
        telemetry.update();

        currentState = State.FIRST_HIGH_POLE;

        while (opModeIsActive()) {
            switch (currentState) {
                case FIRST_HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(FIRST_HIGH_POLE);
                        next(State.STRAFE);
                    }
                    break;

                case STRAFE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(STRAFE);
                        next(State.DROP_FIRST_CONE);
                    }
                    break;

                case DROP_FIRST_CONE:
                    if (!drive.isBusy()) {
                        belt.move(Consts.Belt.CONE_DROP);
                        sleep(1500);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(500);
                        next(State.PREPARE_TO_TURN);
                    }
                    break;

                case PREPARE_TO_TURN:
                    if (!drive.isBusy()) {
                        sleep(1000);
                        belt.move(Consts.Belt.UP);
                        lift.move(Consts.Lift.ZERO);
                        turntable.move(0);

                        drive.followTrajectoryAsync(PREPARE_TO_TURN);

                        next(State.TURN_TO_CONESTACK);
                    }
                    break;

                case TURN_TO_CONESTACK:
                    if (!drive.isBusy()) {
                        drive.turn(Math.toRadians(-90));
                        next(State.GO_TOWARDS_CONESTACK);
                    }
                    break;

                case GO_TOWARDS_CONESTACK:
                    if (!drive.isBusy()) {
                        sleep(1000);
                        lift.move(liftPosition[cyclesCompleted]);
                        turntable.move(0);
                        belt.move(Consts.Belt.DOWN);
                        drive.followTrajectoryAsync(GO_TOWARDS_CONESTACK);
                        next(State.PICKUP_CONE);
                    }
                    break;

                case PICKUP_CONE:
                    if (!drive.isBusy()) {
                        claw.move(Consts.Claw.CLOSECLAW);

                        sleep(1000);
                        lift.move(Consts.Lift.LOW);

                        sleep(200);

                        cyclesCompleted++;
                        next(State.CYCLE_HIGHPOLE);
                    }
                    break;

                case CYCLE_HIGHPOLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(CYCLE_HIGH_POLE);
                        next(State.DROP_CYCLE_CONE);
                    }
                    break;

                case DROP_CYCLE_CONE:
                    if (!drive.isBusy()) {
                        sleep(1000);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(500);
                        belt.move(Consts.Belt.UP);
                        if (cyclesCompleted < 1) {
                            next(State.CYCLE_CONESTACK);
                        } else {
                            belt.move(Consts.Belt.UP);
                            lift.move(Consts.Lift.ZERO);
                            turntable.move(0);
                            claw.move(Consts.Claw.CLOSECLAW);
                            next(State.PARK);
                        }
                    }
                    break;

                case CYCLE_CONESTACK:
                    if (!drive.isBusy()) {
                        lift.move(liftPosition[cyclesCompleted]);
                        turntable.move(0);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(1000);
                        belt.move(Consts.Belt.DOWN);
                        drive.followTrajectoryAsync(CYCLE_CONESTACK);
                        next(State.PICKUP_CONE);
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(PARK);
                        turntable.move(0);
                        lift.move(Consts.Lift.ZERO);
                        claw.move(Consts.Claw.CLOSECLAW);
                        belt.move(Consts.Belt.UP);
                    }

                    next(State.IDLE);
                    break;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("current state", currentState);
            telemetry.addData("busy", drive.isBusy());
            telemetry.addData("cycles ", cyclesCompleted);
            telemetry.addData("belt ", belt.getPosition());
            telemetry.addData("lift ", lift.getPosition());
            telemetry.update();
        }
    }

    // @TODO: 1/14/2023
    // get the translational pids tuned

    // For drivetrain states/trajectories, GO_{FIRST PLACE}_{LAST PLACE}

    enum State {
        FIRST_HIGH_POLE,
        DROP_FIRST_CONE,
        STRAFE,
        PREPARE_TO_TURN,
        TURN_TO_CONESTACK,
        GO_TOWARDS_CONESTACK,
        PICKUP_CONE,
        CYCLE_HIGHPOLE,
        DROP_CYCLE_CONE,
        CYCLE_CONESTACK,
        PARK,
        IDLE
    }
}
