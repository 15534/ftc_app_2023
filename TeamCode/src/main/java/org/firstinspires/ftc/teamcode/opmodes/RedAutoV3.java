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

// Implements TeddyPlan
@Autonomous(name = "RedAutoV3")
@Config
public class RedAutoV3 extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    Pose2d startingPos = new Pose2d(36, -62, Math.toRadians(90));
    ElapsedTime runtime = new ElapsedTime();
    int cyclesCompleted = 0;
    int[] liftPosition = {150, 60, 40, 27, 0};

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    TurnTable turntable = new TurnTable();

    Trajectory firstLowPole,
            firstMedPole,
            firstConeStack,
            coneStack,
            placeHighPole,
            park,
            FIRST_HIGH_POLE;
    Trajectory currentTrajectory;
    TrajectorySequence turnAfterHighPole;
    TrajectorySequence currentTrajectorySequence;

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
                        .lineTo(new Vector2d(36, 0))
                        .addDisplacementMarker(
                                2,
                                () -> {
                                    turntable.turn(90);
                                    lift.moveLift(Constants.LiftTargets.HIGH);
                                })
                        .addDisplacementMarker(
                                10,
                                () -> {
                                    belt.moveBelt(Constants.IntakeTargets.DOWN);
                                })
                        .build();

        Trajectory PREPARE_TO_TURN =
                drive.trajectoryBuilder(FIRST_HIGH_POLE.end())
                        .lineTo(new Vector2d(36, -11))
                        .build();

        Trajectory GO_TOWARDS_CONESTACK =
                drive.trajectoryBuilder(
                                PREPARE_TO_TURN.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                        .lineTo(new Vector2d(44, -11))
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
                        sleep(700);
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        sleep(500);
                        next(State.PREPARE_TO_TURN);
                    }

                case PREPARE_TO_TURN:
                    if (!drive.isBusy()) {
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
                        drive.followTrajectoryAsync(GO_TOWARDS_CONESTACK);

                        next(State.PICKUP_CONE);
                    }

                    break;
                case PICKUP_CONE:
                    lift.moveLift(liftPosition[cyclesCompleted]);
                    belt.moveBelt(Constants.IntakeTargets.DOWN);

                    sleep(2000);

                    claw.moveClaw(Constants.ClawTargets.CLOSECLAW);

                    sleep(2000);

                    belt.moveBelt(Constants.IntakeTargets.UP);

                    lift.moveLift(Constants.LiftTargets.HIGH);

                    next(State.IDLE);

                    break;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

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
        FIRST_HIGH_POLE,
        DROP_FIRST_CONE,
        PREPARE_TO_TURN,
        TURN_TO_CONESTACK,
        GO_TOWARDS_CONESTACK,
        PICKUP_CONE,
        IDLE
    }
}
