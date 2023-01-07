package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpV1 extends LinearOpMode {
    public static double DPAD_SPEED = 0.35;
    public static double BUMPER_ROTATION_SPEED = 0.35;
    public static double ROTATION_MULTIPLIER = 2.05;
    public static boolean TURN_FRONT_BACK = true;
    boolean gp2AReleased = true;
    boolean currentAbtn;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean clawOpen = false;
        double rotation = 0;
//        GamepadEx gp1 = new GamepadEx(gamepad1);
//        GamepadEx gp2 = new GamepadEx(gamepad2);
//        ButtonReader toggleClawButton = new ButtonReader(gp2, GamepadKeys.Button.A);

        Claw claw = new Claw();
        Belt belt = new Belt();
        Lift lift = new Lift();

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        claw.init(hardwareMap);

        while (!isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("right stick x pos", gamepad1.right_stick_x);
            telemetry.addData("right stick y pos", gamepad1.right_stick_y);
            telemetry.addData("rotation", rotation);

            Vector2d translation = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

            if (TURN_FRONT_BACK) {
                rotation = -ROTATION_MULTIPLIER * gamepad1.right_stick_x;
            }
            //            else {
            //                rotation = -ROTATION_MULTIPLIER * gamepad1.right_stick_y;
            //            }

            // slow translation with dpad
            if (gamepad1.dpad_up) {
                translation = new Vector2d(DPAD_SPEED, 0);
            } else if (gamepad1.dpad_down) {
                translation = new Vector2d(-DPAD_SPEED, 0);
            } else if (gamepad1.dpad_left) {
                translation = new Vector2d(0, DPAD_SPEED);
            } else if (gamepad1.dpad_right) {
                translation = new Vector2d(0, -DPAD_SPEED);
            }

            // @TODO: Key map with their corresponding functions
            // GamePad1: Driver only, no external functions
            // GamePad2
            // A: Toggle claw
            // B:
            // X:
            // Y:

            telemetry.addData("gamepad2A", gamepad2.a);
            telemetry.addData("released", gp2AReleased);

            currentAbtn = gamepad2.a;

            if (!currentAbtn){
                gp2AReleased = true;
            }

            if (currentAbtn && gp2AReleased) {
                gp2AReleased = false;
                if (clawOpen) {
                    claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                    clawOpen = false;
                }
                else {
                    claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                    clawOpen = true;
                }
            }

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            drive.setWeightedDrivePower(new Pose2d(translation, rotation));

            telemetry.update();
        }
    }

    public void toggleClaw() {}
}
