package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "BeltTest")
@Config

public class BeltTest extends LinearOpMode {
    public static int BELT_POSITION_START = -280;
    public static int BELT_POSITION_END = -10;
    public static Constants.ClawTargets OPEN_OR_CLOSE = Constants.ClawTargets.CLOSECLAW;

    public Claw myClaw = new Claw();

    @Override
    public void runOpMode() throws InterruptedException {

        myClaw.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {
            myClaw.moveClaw(OPEN_OR_CLOSE);
            myClaw.moveBelt(BELT_POSITION_START);
            myClaw.moveBelt(BELT_POSITION_END);
            telemetry.addData("belt", myClaw.getBeltPosition());
            telemetry.addData("servo", myClaw.getServoPosition());
            telemetry.update();
        }

    }
}