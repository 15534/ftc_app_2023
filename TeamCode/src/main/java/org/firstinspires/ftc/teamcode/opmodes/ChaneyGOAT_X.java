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

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "ChaneyGOAT_X", group = "experimental")
@Config

public class ChaneyGOAT_X extends LinearOpMode {

    enum State {
        HIGH_POLE,
        BACK_A_BIT,
        TO_CONE_STACK,
        LOW_JUNCTION_SCORE,
        TO_GROUND_JUNCTION,
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

    Pose2d startingPos = new Pose2d(36, -62, Math.toRadians(90));
    Vector2d highJunctionPos = new Vector2d(36, 0);
    Vector2d coneStackAlignment = new Vector2d(36, -12);
    Pose2d coneStack = new Pose2d(56, -12.5, Math.toRadians(0));
    Vector2d groundJunction = new Vector2d(46, -12);
    Vector2d parkPosition = new Vector2d();

    int[] liftPosition = {275, 200, 130, 70, 0};
    int lowJunctionCycles = 3;

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
        Trajectory highPole =
                drive.trajectoryBuilder(startingPos)
                        .lineTo(
                                highJunctionPos,
                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addSpatialMarker(new Vector2d(36, -60),
                                () -> {
                                    turnTable.move(90);
                                })
                        .addSpatialMarker(new Vector2d(36, -30),
                                () -> {
                                    lift.move(Consts.Lift.AUTO_HIGH);
                                })
//                        .addSpatialMarker(new Vector2d(36, -5),
//                                () -> {
//                                    belt.move(Consts.Belt.CONE_DROP);
//                                })
                        .build();
        Trajectory backABit =
                drive.trajectoryBuilder(highPole.end())
                        .lineTo(coneStackAlignment)
                        .build();
        Trajectory toConeStack =
                drive.trajectoryBuilder(backABit.end())
                        .lineToLinearHeading(coneStack)
                        .addSpatialMarker(new Vector2d(36, -12),
                                () -> {
                                    lift.move(liftPosition[0]);
                                })
                        .addSpatialMarker(new Vector2d(40, -12),
                                () -> {
                                    turnTable.move(0);
                                })
                        .build();
        Trajectory toGroundJunction =
                drive.trajectoryBuilder(toConeStack.end())
                        .lineTo(groundJunction)
                        .addSpatialMarker(new Vector2d(55, -12),
                                () -> {
                                    lift.move(Consts.Lift.AUTO_GROUND);
                                })
                        .addSpatialMarker(new Vector2d(55, -12),
                                () -> {
                                    turnTable.move(90);
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
                drive.trajectoryBuilder(toGroundJunction.end())
                        .lineTo(parkPosition)
                        .build();

        currentState = State.HIGH_POLE;

        waitForStart();

        while (opModeIsActive()) {
            switch (currentState) {
                case HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(highPole);
                        sleep(250);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(100);
//                        belt.move(Consts.Belt.UP);
                        sleep(100);
                        lift.move(Consts.Lift.MEDIUM);
                        next(State.BACK_A_BIT);
                    }
                    break;
                case BACK_A_BIT:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(backABit);
                        next(State.TO_CONE_STACK);
                    }
                    break;
                case TO_CONE_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(toConeStack);
//                        belt.move(Consts.Belt.DOWN);
                        next(State.LOW_JUNCTION_SCORE);
                    }
                    break;
                case LOW_JUNCTION_SCORE:
                    if (!drive.isBusy()) {

                        int conesCycled = 0;
                        int targetLiftPosition = 1;

                        claw.move(Consts.Claw.CLOSECLAW);
                        sleep(150);

                        while (conesCycled != lowJunctionCycles) {

                            // @TODO: combine lift and turntable movements
                            lift.move(Consts.Lift.AUTO_LOW);
                            sleep(300);

                            turnTable.move(-120);
                            sleep(400);

                            claw.move(Consts.Claw.OPENCLAW);
                            sleep(300);

                            turnTable.move(0);
                            sleep(400);

                            lift.move(liftPosition[targetLiftPosition]);
                            sleep(300);

                            claw.move(Consts.Claw.CLOSECLAW);
                            sleep(300);

                            conesCycled += 1;
                            targetLiftPosition += 1;
                        }

                        sleep(150);
//                        belt.move((Consts.Belt.UP));

                        next(State.TO_GROUND_JUNCTION);
                    }
                    break;
                case TO_GROUND_JUNCTION:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(toGroundJunction);

                        sleep(150);
                        claw.move(Consts.Claw.OPENCLAW);

                        sleep(150);
//                        belt.move((Consts.Belt.UP));

                        next(State.PARK);
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(park);

                        claw.reset();
//                        belt.reset();
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
