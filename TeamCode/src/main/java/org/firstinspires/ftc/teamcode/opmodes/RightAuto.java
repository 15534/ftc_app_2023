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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

// Implements TeddyPlan
@Autonomous(name = "RightAuto")
@Config
public class RightAuto extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    // P: 16, D: 0.14

    Pose2d startingPos = new Pose2d(36, -64, Math.toRadians(90));
    ElapsedTime runtime = new ElapsedTime();
    int cyclesCompleted = 0;
    int[] liftPosition = {290, 200, 130, 70, 0};

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    TurnTable turntable = new TurnTable();

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
                        .lineTo(new Vector2d(35.6, 0))
                        .addDisplacementMarker(
                                1, () -> {
                                    turntable.turn(90);
                                })
                        .addDisplacementMarker(10, ()->{
                            lift.moveLift(Constants.LiftTargets.HIGH);
                        })
                        .build();

        Trajectory PREPARE_TO_TURN =
                drive.trajectoryBuilder(FIRST_HIGH_POLE.end())
                        .lineTo(new Vector2d(36, -12))
                        .build();

        coneStackStartPos = new Pose2d(PREPARE_TO_TURN.end().getX(), PREPARE_TO_TURN.end().getY(), 0);

        Trajectory GO_TOWARDS_CONESTACK =
                //new Pose2d(36, -13, 0) not this
                drive.trajectoryBuilder(coneStackStartPos) // rewrite heading
                        .lineTo(new Vector2d(56, -11.6))
                        .build();

        Trajectory CYCLE_HIGH_POLE =
                drive.trajectoryBuilder(GO_TOWARDS_CONESTACK.end())

                        .lineTo(new Vector2d(22, -12.4),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(
                        1, () -> {
                            turntable.turn(90);
                        })
                        .addDisplacementMarker(10, () -> {
                            turntable.turn(90);
                            lift.moveLift(Constants.LiftTargets.HIGH);
                        })
                        .build();

        Trajectory CYCLE_CONESTACK =
                drive.trajectoryBuilder(CYCLE_HIGH_POLE.end())
                        .lineTo(new Vector2d(56, -11.6))
                        .build();

        runtime.reset();
        waitForStart();

        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        currentState = State.FIRST_HIGH_POLE;

        while (opModeIsActive()) {
            switch (currentState) {
                case FIRST_HIGH_POLE:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(FIRST_HIGH_POLE);
                        next(State.DROP_FIRST_CONE);
                    }
                    break;

                case DROP_FIRST_CONE:
                    if (!drive.isBusy()) {
                        belt.moveBelt(Constants.IntakeTargets.CONE_DROP);
                        sleep(700);
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        sleep(500);
                        next(State.PREPARE_TO_TURN);
                    }
                    break;

                case PREPARE_TO_TURN:
                    if (!drive.isBusy()) {
                        sleep(1000);
                        belt.moveBelt(Constants.IntakeTargets.UP);
                        lift.moveLift(Constants.LiftTargets.PICKUP);
                        turntable.turn(0);

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
                        lift.moveLift(liftPosition[cyclesCompleted]);
                        turntable.turn(0);
                        belt.moveBelt(Constants.IntakeTargets.DOWN);
                        drive.followTrajectoryAsync(GO_TOWARDS_CONESTACK);
                        next(State.PICKUP_CONE);
                    }
                    break;

                case PICKUP_CONE:
                    if (!drive.isBusy()) {
                        claw.moveClaw(Constants.ClawTargets.CLOSECLAW);

                        sleep(1000);
                        lift.moveLift(Constants.LiftTargets.LOW);

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
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        sleep(500);
                        belt.moveBelt(Constants.IntakeTargets.UP);
                        if (cyclesCompleted < 1){
                            next(State.CYCLE_CONESTACK);
                        }
                        else{
                            belt.moveBelt(Constants.IntakeTargets.UP);
                            lift.moveLift(Constants.LiftTargets.PICKUP);
                            turntable.turn(0);
                            claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                            next(State.IDLE);
                        }

                    }
                    break;

                case CYCLE_CONESTACK:
                    if (!drive.isBusy()) {
                        lift.moveLift(liftPosition[cyclesCompleted]);
                        turntable.turn(0);
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        sleep(1000);
                        belt.moveBelt(Constants.IntakeTargets.DOWN);
                        drive.followTrajectoryAsync(CYCLE_CONESTACK);
                        next(State.PICKUP_CONE);
                    }
                    break;

            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("current state", currentState);
            telemetry.addData("busy", drive.isBusy());
            telemetry.addData("cycles ", cyclesCompleted);
            telemetry.addData("belt ", belt.getBeltPosition());
            telemetry.addData("lift ", lift.getPosition());
            telemetry.update();
        }
    }
    // TODO: 1/14/2023
    // get the translational pids tuned

    // For drivetrain states/trajectories, GO_{FIRST PLACE}_{LAST PLACE}
    enum State {
        FIRST_HIGH_POLE,
        DROP_FIRST_CONE,
        PREPARE_TO_TURN,
        TURN_TO_CONESTACK,
        GO_TOWARDS_CONESTACK,
        PICKUP_CONE,
        CYCLE_HIGHPOLE,
        DROP_CYCLE_CONE,
        CYCLE_CONESTACK,
        IDLE
    }
}
