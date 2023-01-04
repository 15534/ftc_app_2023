package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "PickUpTest")
@Config

public class PickUpTest extends LinearOpMode {
    public Lift myLift = new Lift();
    public Claw myClaw = new Claw();
    public Belt myBelt = new Belt();
    public TurnTable myTable = new TurnTable();
    private ElapsedTime myTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        // initing stuff, init does initial movement
        myClaw.init(hardwareMap);
        myLift.init(hardwareMap);
        myBelt.init(hardwareMap);
        myTable.init(hardwareMap);

        // claw closed, lift down, belt up, table at 0

        waitForStart();
        myTimer.reset();

        // table unused for first test
        while (opModeIsActive() && !isStopRequested()) {

            if (myTimer.seconds() > 0 && myTimer.seconds() < 5){
                // step 1: claw open, lift down, belt in pickup, table at 0
                myClaw.moveClaw(Constants.ClawTargets.OPENCLAW);
                myBelt.moveBelt(Constants.IntakeTargets.PICKUP);
            }

            else if (myTimer.seconds() > 5 && myTimer.seconds() < 10) {
                // step 2: claw closed, lift down, belt in pickup, table at 0
                myClaw.moveClaw(Constants.ClawTargets.CLOSECLAW);
            }

            else if (myTimer.seconds() > 10 && myTimer.seconds() < 20) {
                // step 3: claw closed, lift up, belt in hold, table at 180
                myTable.setTablePosition(180);
                myLift.moveLift(Constants.LiftTargets.HIGH);
                myBelt.moveBelt(Constants.IntakeTargets.HOLD);
            }

            else if (myTimer.seconds() > 20 && myTimer.seconds() < 25) {
                // step 4: claw open, lift up, belt in dropoff, table at 180
                myBelt.moveBelt(Constants.IntakeTargets.DROPOFF);
                myClaw.moveClaw(Constants.ClawTargets.OPENCLAW);
            }

            else if (myTimer.seconds() > 25 && myTimer.seconds() < 30) {
                myLift.moveLift(Constants.LiftTargets.MEDIUM);
            }

            else if (myTimer.seconds() > 35 && myTimer.seconds() < 40) {
                myLift.moveLift(Constants.LiftTargets.PUTDOWN);
            }

            myBelt.updateBeltPosition();
            myLift.updateLiftPosition();
            myTable.updateTablePosition();
            // update functions means everything moves asynchronously.

            if (myLift.requestStop) {
                // if lift out of range, STOP
                requestOpModeStop();
            }
        }

    }
}