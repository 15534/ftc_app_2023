package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Consts;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@Autonomous(name = "StationaryGroundLowScore")
@Config
public class StationaryGroundLowScore extends LinearOpMode {

    // facing field side
    //    Pose2d startingPos = new Pose2d(52, -12, Math.toRadians(0));
    int[] liftPosition = {290, 200, 130, 70, 0};

    State currentState = State.IDLE;
    double clawPosition, beltPosition;
    //    SampleMecanumDrive drive;
    ElapsedTime runtime = new ElapsedTime();

    Lift lift = new Lift();
    Claw claw = new Claw();
    Belt belt = new Belt();
    TurnTable turntable = new TurnTable();

    int conesCycled = 0;

    void next(State s) {
        double time = runtime.seconds();
        currentState = s;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //        drive = new SampleMecanumDrive(hardwareMap);
        //        drive.setPoseEstimate(new Pose2d(52, -12, Math.toRadians(0))); // starting Pose2d

        // define trajectories. need none

        runtime.reset();

        waitForStart();

        claw.init(hardwareMap);
        belt.init(hardwareMap);
        turntable.init(hardwareMap);
        lift.init(hardwareMap);

        telemetry.addData("READY", "");
        telemetry.update();

        // state machine time!
        currentState = State.PICK_FROM_CONESTACK;

        while (opModeIsActive()) {
            // constantly recalculating claw position
            clawPosition = Double.parseDouble(String.format("%.2f", claw.clawServo.getPosition())); // rounded to two decimal places

            beltPosition = Double.parseDouble(String.format("%.2f", belt.beltServo.getPosition())); // rounded to two decimal place

            // only have to check current position/is not busy for subsystems moving in previous states. if issues, add turntable checks.

            switch (currentState) { // input to switch
                case PICK_FROM_CONESTACK:
                    // shouldn't need to check lift here
                    if (beltPosition == Consts.BELT_UP_LIMIT && clawPosition == Consts.CLAW_CLOSE_LIMIT) {
                        lift.move(liftPosition[conesCycled]);
                        claw.move(Consts.Claw.OPENCLAW);
                        sleep(500);
                        belt.move(Consts.Belt.DOWN);
                        next(State.REMOVE_FROM_CONESTACK);
                    }
                    break;

                case REMOVE_FROM_CONESTACK:
                    if (!lift.left.isBusy() && !lift.right.isBusy() &&
                            beltPosition == Consts.BELT_DOWN_LIMIT && clawPosition == Consts.CLAW_OPEN_LIMIT) {
                        claw.move(Consts.Claw.CLOSECLAW);
                        sleep(500); // to tighten around cone
                        belt.move(Consts.Belt.UP);
                        next(State.SCORE_LOW);
                    }
                    break;

                case RAISE_LIFT_TO_SCORE:
                    if (beltPosition == Consts.BELT_UP_LIMIT && clawPosition == Consts.CLAW_CLOSE_LIMIT) {
                        turntable.move(-120); // tuned lol
                        lift.move(Consts.Lift.LOW);
                        belt.move(Consts.Belt.CONE_DROP);
                        next(State.SCORE_LOW);
                    }
                    break;

                case SCORE_LOW:
                    // lift is done moving. already confirmed claw in right place
                    if (!lift.left.isBusy() && !lift.right.isBusy() && beltPosition == Consts.BELT_DROP_LIMIT) {
                        claw.move(Consts.Claw.OPENCLAW);
                        conesCycled++;
                        sleep(100); // sometimes claw doesn't open though it should
                        next(State.RESET);
                    }

                case RESET:
                    // checking claw in right position, because only one that moved in previous state
                    if (clawPosition == Consts.CLAW_OPEN_LIMIT) {
                        turntable.move(0);
                        claw.move(Consts.Claw.CLOSECLAW);
                        belt.move(Consts.Belt.UP);
                        next(State.PICK_FROM_CONESTACK);
                    }
                    break;
            }

            telemetry.addData("current state", currentState);
            telemetry.addData("lift position", lift.getPosition());

            telemetry.addData("formatted belt position", beltPosition);
            telemetry.addData("belt position", belt.getPosition());

            telemetry.addData("formatted claw position", clawPosition);
            telemetry.addData("claw position", claw.getPosition());

            telemetry.update();
        }
    }

    // why define here? not earlier?

    enum State {
        RESET,
        REMOVE_FROM_CONESTACK,
        PICK_FROM_CONESTACK,
        RAISE_LIFT_TO_SCORE,
        SCORE_LOW,
        IDLE
    }
}
