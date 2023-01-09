package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpV1 extends LinearOpMode {
    public static double DPAD_SPEED = 0.35;
    public static double BUMPER_ROTATION_SPEED = 0.35;
    public static double ROTATION_MULTIPLIER = 2.05;
    public static boolean TURN_FRONT_BACK = true;
    public static double turntableSensitivity = 1.2;
    boolean gp2AReleased = true;
    boolean gp2BReleased = true;
    boolean currentAbtn;
    boolean currentBbtn;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean clawOpen = false;
        boolean beltUp = true;
        double rotation = 0;
        double tableRotation = 0;

        double beltUpPos = Constants.BELT_POSITION_END;
        double beltDownPos = Constants.BELT_POSITION_START;

        //        GamepadEx gp1 = new GamepadEx(gamepad1);
        //        GamepadEx gp2 = new GamepadEx(gamepad2);
        //        ButtonReader toggleClawButton = new ButtonReader(gp2, GamepadKeys.Button.A);

        Claw claw = new Claw();
        Belt belt = new Belt();
        Lift lift = new Lift();
        TurnTable turntable = new TurnTable();

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);
        //        belt.setBeltPosition(beltDownPos);

        while (!isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
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

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            // @TODO: Key map with their corresponding functions
            // GamePad1: Driver only, no external functions
            // GamePad2
            // A: Toggle claw
            // B: Toggle belt
            // X:
            // Y:
            // dpad up: slides max height

            telemetry.addData("gamepad2A", gamepad2.a);
            telemetry.addData("released", gp2AReleased);

            currentAbtn = gamepad2.a; // for toggling claw
            currentBbtn = gamepad2.b; // for toggling belt

            if (!currentAbtn) {
                gp2AReleased = true;
            }

            if (currentAbtn && gp2AReleased) {
                gp2AReleased = false;
                if (clawOpen) {
                    claw.moveClaw(Constants.ClawTargets.CLOSECLAW);
                    clawOpen = false;
                } else {
                    claw.moveClaw(Constants.ClawTargets.OPENCLAW);
                    clawOpen = true;
                }
            }

            if (!currentBbtn) {
                gp2BReleased = true;
            }

            if (currentBbtn && gp2BReleased) {
                gp2BReleased = false;
                if (beltUp) {
                    belt.setBeltPosition(beltDownPos);
                    beltUp = false;
                } else {
                    belt.setBeltPosition(beltUpPos);
                    beltUp = true;
                }
            }

            tableRotation += (turntableSensitivity * gamepad2.right_stick_x);
            if (tableRotation >= 270) {
                tableRotation = 270;
            }

            if (tableRotation <= -270) {
                tableRotation = -270;
            }

            // 279
            turntable.turn(tableRotation);

            // moving linear slides

            // mappings
            // dpad_up -> high
            // dpad_left -> mid
            // dpad_right -> low
            // dpad_down -> all the back to 0

            if (gamepad2.dpad_up) {
                lift.moveLift(Constants.LiftTargets.HIGH);
            } else if (gamepad2.dpad_right) {
                lift.moveLift(Constants.LiftTargets.LOW);
            } else if (gamepad2.dpad_left) {
                lift.moveLift(Constants.LiftTargets.MEDIUM);
            } else if (gamepad2.dpad_down) {
                lift.moveLift(Constants.LiftTargets.PICKUP);
            }

            // drive and subsystem updates
            drive.setWeightedDrivePower(new Pose2d(translation, rotation));
            belt.updateBeltPosition();
            lift.updateLiftPosition();

            // telemetry updates
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("right stick x pos", gamepad1.right_stick_x);
            telemetry.addData("right stick y pos", gamepad1.right_stick_y);
            telemetry.addData("rotation", rotation);
            telemetry.addData("Belt Position", belt.getPosition());
            telemetry.addData("Lift Position", lift.getPosition());
            telemetry.addData("Dpad Up", gamepad2.dpad_up);
            telemetry.addData("Dpad right", gamepad2.dpad_right);
            telemetry.addData("Dpad down", gamepad2.dpad_down);
            telemetry.addData("Dpad left", gamepad2.dpad_left);

            telemetry.update();

        }
    }

    public void toggleClaw() {}
}
