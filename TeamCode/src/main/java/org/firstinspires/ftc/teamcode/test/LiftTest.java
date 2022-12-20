package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "LiftTest")
@Config

public class LiftTest extends LinearOpMode {
    public static double POWER = 20;

    public static Constants.LiftTargets LIFT_HEIGHT = Constants.LiftTargets.MEDIUM;
    public Lift myLift = new Lift(POWER);

    @Override
    public void runOpMode() throws InterruptedException {

        myLift.init(hardwareMap);

        waitForStart();

        myLift.goTo(LIFT_HEIGHT);

        while (opModeIsActive()) {
            myLift.update();
        }

    }
}