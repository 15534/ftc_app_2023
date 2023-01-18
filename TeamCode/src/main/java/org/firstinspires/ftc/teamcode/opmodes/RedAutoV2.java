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

    Trajectory firstHighPole, firstCycleAlign, turnToConeStack, firstConeStack, cycleHighPole, cycleConeStack, park;
    Trajectory currentTrajectory;
    TrajectorySequence turnAfterHighPole;
    TrajectorySequence currentTrajectorySequence;


    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public void buildTrajectories() {


        firstHighPole = drive.trajectoryBuilder(startingPos).addSpatialMarker(new Vector2d(36, 0), () -> {
            claw.moveClaw(Constants.ClawTargets.OPENCLAW);
        }).addDisplacementMarker(() -> {
            turntable.turn(0);
        }).lineToConstantHeading(new Vector2d(36, 0)).build();

        firstCycleAlign = drive.trajectoryBuilder(firstHighPole.end()).addSpatialMarker(new Vector2d(36, 0), () -> {
            claw.moveClaw(Constants.ClawTargets.OPENCLAW);
        }).addDisplacementMarker(2, () -> {
            turntable.turn(0);
            lift.moveLift(Constants.LiftTargets.PICKUP);
        }).lineTo(new Vector2d(36, -12)).build();


        firstConeStack = drive.trajectoryBuilder(firstCycleAlign.end()).addSpatialMarker(new Vector2d(36, 0), () -> {
            claw.moveClaw(Constants.ClawTargets.OPENCLAW);
        }).addDisplacementMarker(2, () -> {
            turntable.turn(0);
            lift.moveLift(Constants.LiftTargets.PICKUP);
        }).lineTo(new Vector2d(62, -12)).build();


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

        currentState = State.PRELOAD_SCORE_HIGH;

        while (opModeIsActive()) {
            telemetry.addLine("running");

            // @TODO: Determine cycle-time using "elapsed" variable
            //            double elapsed = runtime.seconds() - time;

            switch (currentState) {
                case PRELOAD_SCORE_HIGH:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstHighPole);
                        lift.moveLift(Constants.LiftTargets.HIGH);
                        belt.moveBelt(Constants.IntakeTargets.DROPOFF);
                        next(State.FIRST_CYCLE_ALIGN);

                    }
                    break;

                case FIRST_CYCLE_ALIGN:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstCycleAlign);

                        sleep(750);
                        belt.moveBelt(Constants.IntakeTargets.HOLD);
                        lift.moveLift(Constants.LiftTargets.PICKUP);
                        turntable.turn(-90);
                        sleep(1000);
                        belt.moveBelt(Constants.IntakeTargets.PICKUP);
                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);

                        next(State.FIRST_CONESTACK);
                    }
                    break;

                case TURN_TO_CONESTACK:
                    if(!drive.isBusy()) {
                        drive.turn(-90);

                        next(State.FIRST_CONESTACK);
                    }
                case FIRST_CONESTACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstConeStack);
                        //add claw close, lift up, belt open

                        next(State.IDLE);
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
        PRELOAD_SCORE_HIGH, FIRST_CYCLE_ALIGN, TURN_TO_CONESTACK, FIRST_CONESTACK, GO_HIGHJUNC, GO_CONESTACK, PARK, IDLE
    }
}
