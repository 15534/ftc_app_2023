package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "LiftTest")
@Config

public class LiftTest extends LinearOpMode {
    public Lift myLift = new Lift();

    @Override
    public void runOpMode() throws InterruptedException {

        myLift.init(hardwareMap);

        waitForStart();

        myLift.goTo(Constants.LiftTargets.MEDIUM);

        while (opModeIsActive()) {
            myLift.update();
        }

    }
}