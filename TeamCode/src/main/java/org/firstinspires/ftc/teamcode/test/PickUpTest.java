package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "PickUpTest")
@Config

public class PickUpTest extends LinearOpMode {
    public Lift myLift = new Lift(20);
    public Claw myClaw = new Claw(hardwareMap);

    // place claw at very top
    // place turntable at 0 degrees

    @Override
    public void runOpMode() throws InterruptedException {

        // initing stuff
        myLift.init(hardwareMap); // init does initial movement


        waitForStart();
        myLift.goTo(Constants.LiftTargets.PICKUP);

        while (opModeIsActive()) {
            myLift.goTo(Constants.LiftTargets.HIGH);
            myLift.update();

//            myClaw.pickUpCone();

//            myClaw.dropOffCone();
        }

    }
}