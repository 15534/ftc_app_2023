package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

/*
 * Key Map
 * Gamepad 1
 * Left joystick: Translation
 * DPad: Slow translation
 * Right joystick: Rotation
 * Bumpers: Slow rotation
 * -----------------------
 * Gamepad 2
 * A: Toggle Claw
 * B: Toggle Belt
 * DPad up: Move lift to high
 * DPad left: Move lift to mid
 * DPad right: Move lift to low
 * DPad down: Move lift to zero
 * Right joystick: Turntable rotation
 * Left joystick: manual control of lift from 0-300
 */

@TeleOp(name = "DeBruh")
@Config
public class DeBruh extends LinearOpMode {
    public static double ROTATION_LOCK_GAIN = 0.02;
    public static double ROTATION_LOCK_MULTIPLIER = 1.8;
    private double robotHeading = 0;
    private double headingOffset = 90;
    private double headingError = 0;
    private double targetHeading = 0;
    public static double DEFAULT_MOVE_MULTIPLIER = .64;
    public static double SLOW_MOVEMENT_MULTIPLIER = .36;
    public static double FAST_MOVEMENT_MULTIPLIER = 1;
    public static double ROTATION_MULTIPLIER = -1.9;
    public static double SLOW_ROTATION_MULTIPLIER = .27;
    public static boolean TURN_X_JOYSTICK = true;
    public static double turntableSensitivity = 2.2;
    boolean gp2AReleased = true;
    boolean gp2BReleased = true;
    boolean gp2YReleased = true;
    boolean gp2XReleased = true;
    boolean gp2RBumperReleased = true;
    boolean gp2LBumperReleased = true;
    boolean rightTriggerRelased;
    boolean currentBbtn, currentYbtn, currentXbtn;
    boolean currentRBumper;
    boolean currentLBumper;
    int currentIndex = 4;
    boolean gp2LeftStickYJustPressed = false;
    boolean liftDown = true;
    int[] conePositions = { 290, 200, 130, 70, 0 };

    double movementHorizontal = 0;
    double movementVertical = 0;
    private BNO055IMU imu;

    private Vector2d translation;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        boolean clawOpen = false;
        boolean beltUp = true;
        double rotation = 0;
        double tableRotation = 0;
        double movementRotation = 0;
        double lastRight = 0;
        double lastLeft = 0;

        Claw claw = new Claw();
        Belt belt = new Belt();
        Lift lift = new Lift();
        TurnTable turntable = new TurnTable();
        Consts.Belt beltTarget = Consts.Belt.UP;

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        while (!isStopRequested() && !lift.requestStop) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            // Dpad motion in one of four directions
            if (gamepad2.left_stick_y<-.2 && gamepad2.a){
                lift.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.left.setPower(-.1);
                lift.right.setPower(.1);
                lastLeft = lift.left.getCurrentPosition();
                lastRight = lift.right.getCurrentPosition();
                sleep(100);
                while(gamepad1.a){
                    lastRight = lift.right.getCurrentPosition();
                    lastLeft = lift.left.getCurrentPosition();
                    if (lift.right.getCurrentPosition() == lastRight || lift.left.getCurrentPosition()==lastLeft){
                        break;
                    }
                }

                lift.left.setPower(0);
                lift.right.setPower(0);
                lift.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.dpad_down
                    || gamepad1.dpad_up
                    || gamepad1.dpad_left
                    || gamepad1.dpad_right) {
                movementRotation = 0;
                if (gamepad1.dpad_down) {
                    movementVertical = .8;
                    movementHorizontal = 0;
                } else if (gamepad1.dpad_up) {
                    movementVertical = -.8;
                    movementHorizontal = 0;
                } else if (gamepad1.dpad_right) {
                    movementVertical = 0;
                    movementHorizontal = .8;

                } else if (gamepad1.dpad_left) {
                    movementVertical = 0;
                    movementHorizontal = -.8;
                }
            } else { // joystick use instead, movement in vector directions. can explicitly define
                // if needed.
                movementVertical = gamepad1.left_stick_y;
                movementHorizontal = gamepad1.left_stick_x;
                movementRotation = gamepad1.right_stick_x;
            }
            if (TURN_X_JOYSTICK) {
                rotation = ROTATION_MULTIPLIER * movementRotation;
            }

            if (gamepad1.left_trigger > .3) {
                translation = new Vector2d(
                        -1 * SLOW_MOVEMENT_MULTIPLIER * movementVertical,
                        -1 * SLOW_MOVEMENT_MULTIPLIER * movementHorizontal);
                rotation = rotation * SLOW_ROTATION_MULTIPLIER;
            }

            else if (gamepad1.right_trigger > .3) {
                translation = new Vector2d(
                        -1 * movementVertical,
                        -1 * movementHorizontal);
                rotation = rotation;
            } else {
                translation = new Vector2d(
                        -1 * DEFAULT_MOVE_MULTIPLIER * movementVertical,
                        -1 * DEFAULT_MOVE_MULTIPLIER * movementHorizontal);
            }

            // xbutton for moving lift to conestack heights
            if (Math.abs(gamepad2.left_stick_y) > 0 && lift.getPosition() <= 1900 && !gamepad1.a) {
                int movePosition = (int) (lift.getPosition() +  -1 * gamepad2.left_stick_y * 75);
                if (movePosition > 1900) {
                    movePosition = 1900;
                } else if (movePosition < 0) {
                    movePosition = 0;
                }
                lift.move(movePosition);
            }

            currentXbtn = gamepad2.x;
            if (!currentXbtn) {
                gp2XReleased = true;
            }
            if (currentXbtn && gp2XReleased) {
                gp2XReleased = false;
                currentIndex = currentIndex - 1;
                if (currentIndex < 0) {
                    currentIndex = 4;
                }
                lift.move(conePositions[currentIndex]);
            }

            currentBbtn = gamepad2.b;
            if (!currentBbtn) {
                gp2BReleased = true;
            }

            // Toggle claw
            rightTriggerRelased = gamepad2.right_trigger > 0;
            if (!rightTriggerRelased) {
                gp2AReleased = true;
            }

            if (rightTriggerRelased && gp2AReleased) {
                gp2AReleased = false;
                if (clawOpen) {
                    claw.move(Consts.Claw.CLOSECLAW);
                    clawOpen = false;
                } else {
                    claw.move(Consts.Claw.OPENCLAW);
                    clawOpen = true;
                }
            }

            // Toggle belt
            currentBbtn = gamepad2.b;
            if (!currentBbtn) {
                gp2BReleased = true;
            }

            if (currentBbtn && gp2BReleased) {
                gp2BReleased = false;
                if (beltUp) {
                    if (liftDown) { // handles case where belt is going down preemptively as you
                        // prepare to pick up cone
                        claw.move(Consts.Claw.OPENCLAW);
                    }
                    if (liftDown) {
                        belt.move(Consts.Belt.CONE_DROP);
                    } else {
                        belt.move(Consts.Belt.DOWN);
                    }

                    beltUp = false;
                } else {
                    belt.move(Consts.Belt.UP);
                    beltUp = true;
                }
            }

            // y for ground junction - moving belt down w/o claw down
            if (gamepad2.y) {
                belt.move(Consts.Belt.DOWN);
                beltUp = false;
            }

            // Turntable rotation
            currentRBumper = gamepad2.left_bumper; // flipped on purpouse
            if (!currentRBumper) {
                gp2RBumperReleased = true;
            }

            if (currentRBumper && gp2RBumperReleased) {
                if (!beltUp && lift.getTarget() == 0) {
                    belt.move(Consts.Belt.UP);
                    beltUp = true;
                }

                gp2RBumperReleased = false;
                if (tableRotation < -90) {
                    tableRotation = -90;
                } else if (tableRotation < 0) {
                    tableRotation = 0;
                } else if (tableRotation < 90) {
                    tableRotation = 90;
                } else if (tableRotation < 180) {
                    tableRotation = 180;
                }
            }

            currentLBumper = gamepad2.right_bumper; // flipped on purpouse
            if (!currentLBumper) {
                gp2LBumperReleased = true;
            }

            if (currentLBumper && gp2LBumperReleased) {
                if (!beltUp && lift.getTarget() == 0) {
                    belt.move(Consts.Belt.UP);
                    beltUp = true;
                }

                gp2LBumperReleased = false;
                if (tableRotation > 90) {
                    tableRotation = 90;
                } else if (tableRotation > 0) {
                    tableRotation = 0;
                } else if (tableRotation > -90) {
                    tableRotation = -90;
                } else if (tableRotation > -180) {
                    tableRotation = -180;
                }
            }

            tableRotation += (turntableSensitivity * -gamepad2.right_stick_x);

            // Moving lift
            //commented stuff is movement cancel logic, bad and other stuff
//            if (!lift.left.isBusy() && !lift.right.isBusy()){
                if (gamepad2.dpad_up) {
                    lift.move(Consts.Lift.HIGH);
                    liftDown = false;
                } else if (gamepad2.dpad_left) {
                    lift.move(Consts.Lift.LOW);
                    liftDown = false;
                } else if (gamepad2.dpad_right) {
                    lift.move(Consts.Lift.MEDIUM);
                    liftDown = false;
                } else if (gamepad2.dpad_down) {
                    // belt.moveBelt(Constants.IntakeTargets.PICKUP);
                    lift.move(Consts.Lift.ZERO);
                    liftDown = true;
                }
//            }


            // slow Manual lift control with left joystick. Slightly dysfunctional.

            // if (Math.abs(gamepad2.left_stick_y) > 0.3) {
            // gp2LeftStickYJustPressed = true;
            // }
            //
            // if (Math.abs(gamepad2.left_stick_y) > 0.3) {
            // // joystick going down
            // // go to top of conestacks
            // // lift.setLiftPosition(conePositions[0]);
            // gp2LeftStickYJustPressed = false;
            // }
            //
            // if (Math.abs(gamepad2.left_stick_y) > 0.3 && !gp2LeftStickYJustPressed) {
            // lift.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            // lift.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //
            // lift.left.setPower(-0.5 * gamepad2.left_stick_y);
            // lift.right.setPower(0.5 * gamepad2.left_stick_y);
            //
            // // so that this doesn't happen again when we press left stick y after
            // //releasing
            // gp2LeftStickYJustPressed = true;
            // }

            // X: reset subsystems for intaking action
            // turn table turned
            // lift up
            // claw open
            // belt is down
            // belt up -> claw close -> turntable turn back -> lift down

            if (gamepad2.a && Math.abs(gamepad2.left_stick_y) < .1) {
                belt.move(Consts.Belt.UP);
                tableRotation = 0;
                lift.move(Consts.Lift.ZERO);
                liftDown = true;
            }
            if (gamepad1.a) {
                resetHeading();
            }

            if (tableRotation >= 180) {
                tableRotation = 180;
            }
            if (tableRotation <= -180) {
                tableRotation = -180;
            }

            turntable.move(tableRotation);

            if (gamepad1.right_bumper) {
                rotation = getSteeringCorrection(0, ROTATION_LOCK_GAIN) * ROTATION_LOCK_MULTIPLIER;
            }

            drive.setWeightedDrivePower(new Pose2d(translation, rotation));

            telemetry.addData("rTrigger ", gamepad1.right_trigger);
            telemetry.addData("lTrigger ", gamepad1.left_trigger);
            telemetry.addData("clawOpen", clawOpen);
            telemetry.addData("claw position ", claw.getPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("right stick x pos", gamepad1.right_stick_x);
            telemetry.addData("right stick y pos", gamepad1.right_stick_y);
            telemetry.addData("rotation", rotation);
            telemetry.addData("Belt Position", belt.getPosition());
            telemetry.addData("Drift ", belt.drift);
            telemetry.addData("Lift Position", lift.getPosition());
            telemetry.addData("Dpad Up", gamepad2.dpad_up);
            telemetry.addData("Dpad right", gamepad2.dpad_right);
            telemetry.addData("Dpad down", gamepad2.dpad_down);
            telemetry.addData("Dpad left", gamepad2.dpad_left);
            telemetry.addData("gamepad 2 x button", gamepad2.x);
            telemetry.addData("conestack position", currentIndex);
            telemetry.addData("left joystick", gamepad2.left_stick_y);
            telemetry.addData("turn table position", turntable.getPosition());
            telemetry.addData("translation ", translation);
            telemetry.update();
        }
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        robotHeading = getRawHeading() - headingOffset;

        headingError = targetHeading - robotHeading;

        while (headingError > 180)
            headingError -= 360;
        while (headingError <= -180)
            headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
