package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@TeleOp(name = "AsyncTest")
@Config
public class AsyncTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnTable ourTurntable = new TurnTable();
        ourTurntable.init(hardwareMap);

        Trajectory trajectoryForward =
                drive.trajectoryBuilder(new Pose2d())
                        .forward(20)
                        .addDisplacementMarker(() -> ourTurntable.turn(180))
                        .build();
        Trajectory trajectoryBack =
                drive.trajectoryBuilder(new Pose2d())
                        .back(13)
                        .build();

        waitForStart();
        drive.followTrajectory(trajectoryForward);
        while (drive.isBusy()){
            //ourTurntable.turn((180));
            continue;
        }
        drive.followTrajectory(trajectoryBack);
        // Git plagarise: Eric
        // https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/72930-asynchronous-multi-threading-functions-in-autonomous
        // complex  example: https://gist.github.com/David10238/bb7fdbfa710eae0264acd6291ee99cbc
        //         Thread driveForwardThread = new Thread(() ->
        // drive.followTrajectory(trajectoryForward));
        //         Thread driveTurnThread = new Thread(() -> drive.turn(Math.toRadians(180)));
        //         Thread turnTableThread = new Thread(() -> ourTurntable.turn(180));
        //
        //         driveForwardThread.start();
        //         turnTableThread.start();
        // End Git praise: Eric

        while (opModeIsActive()) {}
    }
}

/*
package net.frogbots.roverruckus.opmodes.extended.auto;

import net.frogbots.roverruckus.meta.statemachine.StateMachineImpl;

public class DeployMarkerStateMachine extends StateMachineImpl<DeployMarkerStateMachine.State>
{
    @Override
    public String getName()
    {
        return "DeployMarkerStateMachine";
    }

    enum State
    {
        START,
        LOWER_INTAKE,
        SPIT_MARKER,
        RAISE_INTAKE,
        END
    }

    DeployMarkerStateMachine()
    {
        state = State.START;
    }

    @Override
    public ReturnState runIteration()
    {
        switch (state)
        {
            case START:
            {
                switchState(State.LOWER_INTAKE);
                break;
            }

            case LOWER_INTAKE:
            {
                robot.intake.setFlipPower(-1, true);

                if(robot.intake.getFlipPosition() < robot.intake.MIN_ENCODER_COUNT)
                {
                    robot.intake.setFlipPower(0, true);
                    switchState(State.SPIT_MARKER);
                }

                break;
            }

            case SPIT_MARKER:
            {
                if(getElapsedStateTime() < 1000)
                {
                    robot.intake.reverseNFT();
                }
                else
                {
                    robot.intake.stopNFT();
                    switchState(State.RAISE_INTAKE);
                }
                break;
            }

            case RAISE_INTAKE:
            {
                robot.intake.commandPidRaise();
                switchState(State.END);

                robot.intake.setFlipPower(1, true);

                if(robot.intake.getFlipPosition() > robot.intake.TRANSFER_ENCODER_COUNT)
                {
                    robot.intake.setFlipPower(0, true);
                    switchState(State.END);
                }

                break;
            }

            case END:
            {
                return ReturnState.PROCEED;
            }
        }

        return ReturnState.KEEP_RUNNING_ME;
    }
}
*/
