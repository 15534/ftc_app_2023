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
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "BlueAutoV2")
@Config
public class BlueAutoV2 extends LinearOpMode {

    State currentState = State.IDLE;
    SampleMecanumDrive drive;

    Pose2d startingPos = new Pose2d(12, 62, Math.toRadians(-90));
    ElapsedTime runtime = new ElapsedTime();
    int cyclesCompleted = 0;

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    TurnTable turntable = new TurnTable();

    Trajectory firstHighPole, firstConeStack, coneStack, placeHighPole, park;

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
                        .lineToSplineHeading(new Pose2d(12, 12, 0))
                        .build();

        firstConeStack =
                drive.trajectoryBuilder(firstHighPole.end())
                        .lineTo(new Vector2d(52, 12))
                        .build();

        placeHighPole =
                drive.trajectoryBuilder(firstConeStack.end()).lineTo(new Vector2d(20, 12)).build();

        coneStack =
                drive.trajectoryBuilder(placeHighPole.end())
                        .splineTo(new Vector2d(52, 12), Math.toRadians(0))
                        .build();

        park =
                drive.trajectoryBuilder(placeHighPole.end())
                        .splineToConstantHeading(new Vector2d(10, 35), Math.toRadians(90))
                        .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(-90)));

        buildTrajectories();

        runtime.reset();
        waitForStart();

        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        currentState = State.GO_SUBSTATION_HIGHJUNC;

        // TODO: lift while driving to first traj
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

//                        belt.moveBelt(Constants.IntakeTargets.DROPOFF);

                        long t = System.currentTimeMillis();
                        long end = t + 1250;
//
//                        while (System.currentTimeMillis() < end) {
//                            belt.updateBeltPosition();
//                        }

//                        claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                        sleep(2000);
                        next(State.IDLE);
                    }
                    break;
                case FIRST_CONESTACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(firstConeStack);
                        next(State.IDLE);
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
                            next(State.GO_CONESTACKS_HIGHJUNC);
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

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("current state", currentState);
            telemetry.update();
        }
    }

    // For drivetrain states/trajectories, GO_{FIRST PLACE}_{LAST PLACE}
    enum State {
        GO_SUBSTATION_HIGHJUNC,
        DROP_FIRST_CONE,
        FIRST_CONESTACK,
        GO_HIGHJUNC_CONESTACKS,
        PLACE_HIGHJUNC_CONE,
        GO_CONESTACKS_HIGHJUNC,
        PARK,
        IDLE
    }
}
