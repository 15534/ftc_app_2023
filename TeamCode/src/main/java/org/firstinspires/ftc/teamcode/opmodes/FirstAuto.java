package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

@Autonomous(name = "FirstAuto")
public class FirstAuto extends LinearOpMode {
    public static double DISTANCE = 50;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Trajectory trajectoryForward =
                drive.trajectoryBuilder(new Pose2d()).forward(DISTANCE).build();

        Trajectory trajectoryBackward =
                drive.trajectoryBuilder(trajectoryForward.end()).back(DISTANCE).build();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
        }
    }
}
