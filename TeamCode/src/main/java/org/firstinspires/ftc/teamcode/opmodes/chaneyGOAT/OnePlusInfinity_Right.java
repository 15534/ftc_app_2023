package org.firstinspires.ftc.teamcode.opmodes.chaneyGOAT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "OnePlusInfinity_Right", group = "ChaneyTheGOAT")
@Config
public class OnePlusInfinity_Right extends LinearOpMode {

    enum State {
        IDLE,
        FIRST
    }

    State currentState = State.FIRST;
    SampleMecanumDrive drive;
    Camera camera = new Camera();
    Claw claw = new Claw();
    Belt belt = new Belt();

    Pose2d startingPos = new Pose2d(36, -62, Math.toRadians(90));
    ElapsedTime runTime = new ElapsedTime();

    void next(State s) {
        time = runTime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPos);

        camera.init(hardwareMap);
        claw.init(hardwareMap);
        belt.init(hardwareMap);

        Trajectory FORWARDS =
                drive.trajectoryBuilder(startingPos)
                        .lineTo(
                                new Vector2d(36, 20),
                                SampleMecanumDrive.getVelocityConstraint(
                                        60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(
                                        60)) // Maximum acceleration must be tweaked
                        .build();

        waitForStart();

        while (opModeIsActive()) {
            switch (currentState) {
                case FIRST:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(FORWARDS);
                    }
                    next(State.IDLE);
                    break;
            }
        }
    }
}
