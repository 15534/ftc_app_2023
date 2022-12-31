package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Belt;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Constants;

@TeleOp(name = "BeltTest")
@Config

public class BeltTest extends LinearOpMode {
    public static int beltPosition = 0;
    public static int BELT_POSITION_START = -280;
    public static int BELT_POSITION_END = -10;
    public static Constants.ClawTargets OPEN_OR_CLOSE = Constants.ClawTargets.CLOSECLAW;

    public Belt myBelt = new Belt();

    @Override
    public void runOpMode() throws InterruptedException {

        myBelt.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
//            myBelt.belt.setTargetPosition(0);  //-280
//            myBelt.belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            myBelt.belt.setPower(0.5);
//            telemetry.addData("position: ", myBelt.getPosition());
//            telemetry.update()
            myBelt.setBeltPosition(beltPosition);
            myBelt.updateBeltPosition();
        }

    }
}