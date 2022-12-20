package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "FirstAuto")
public class FirstAuto extends LinearOpMode {
    public static double DISTANCE = 125.25; // inches

    public void runOpMode() throws InterruptedException {
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
