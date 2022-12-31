package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    @Override
    public void runOpMode() throws InterruptedException {

        // initing stuff, init does initial movement
        myClaw.init(hardwareMap);
        myLift.init(hardwareMap);
        myBelt.init(hardwareMap);
        myTable.init(hardwareMap);

        // claw closed, lift down, belt up, table at 0

        waitForStart();
        while (opModeIsActive()) {
            // table unused for first test
            // step 1: claw open, lift down, belt in pickup
            myClaw.moveClaw(Constants.ClawTargets.OPENCLAW);
            myBelt.moveBelt(Constants.IntakeTargets.PICKUP);
            sleep(2000);

            // step 2: claw closed, lift down, belt in pickup
            myClaw.moveClaw(Constants.ClawTargets.CLOSECLAW);
            sleep(2000);

            // step 3: claw closed, lift up, belt in hold
            myLift.moveLift(Constants.LiftTargets.HIGH);
            myBelt.moveBelt(Constants.IntakeTargets.HOLD);
            sleep(2000);

            // step 4: claw open, lift up, belt in dropoff
            myBelt.moveBelt(Constants.IntakeTargets.DROPOFF);
            myClaw.moveClaw(Constants.ClawTargets.OPENCLAW);

        }

    }
}