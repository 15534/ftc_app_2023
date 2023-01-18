package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.OldBelt;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "PickUpTest")
@Config
public class PickUpTest extends LinearOpMode {
    public Lift myLift = new Lift();
    public Claw myClaw = new Claw();
    public OldBelt myOldBelt = new OldBelt();
    public TurnTable myTable = new TurnTable();
    private ElapsedTime myTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        myClaw.init(hardwareMap);
        myLift.init(hardwareMap);
        myOldBelt.init(hardwareMap);
        myTable.init(hardwareMap);

        // After init: claw closed, lift down, belt up, turntable oriented at 0

        waitForStart();
        myTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            if (myTimer.seconds() > 0 && myTimer.seconds() < 5) {
                // Step 1: claw open, lift down, belt in pickup, table at 0
                myClaw.moveClaw(Constants.ClawTargets.OPENCLAW);
                myOldBelt.moveBelt(Constants.IntakeTargets.UP);
            } else if (myTimer.seconds() > 5 && myTimer.seconds() < 10) {
                // Step 2: claw closed, lift down, belt in pickup, table at 0
                myClaw.moveClaw(Constants.ClawTargets.CLOSECLAW);
            } else if (myTimer.seconds() > 10 && myTimer.seconds() < 20) {
                // Step 3: claw closed, lift up, belt in hold, table at 180
                myLift.moveLift(Constants.LiftTargets.HIGH);
                myOldBelt.moveBelt(Constants.IntakeTargets.HOLD);
            } else if (myTimer.seconds() > 25 && myTimer.seconds() < 30) {
                myLift.moveLift(Constants.LiftTargets.MEDIUM);
            }

            // Update functions means everything moves asynchronously.
            myTable.updateTablePosition();

            if (myLift.requestStop) {
                // If lift out of range, STOP
                requestOpModeStop();
            }
        }
    }
}
