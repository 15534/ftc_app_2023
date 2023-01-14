package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OldBelt;

@TeleOp(name = "BeltTest")
@Config()
public class BeltTest extends LinearOpMode {
    public static int beltPosition = 0;

    public OldBelt myOldBelt = new OldBelt();

    @Override
    public void runOpMode() throws InterruptedException {

        myOldBelt.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
//            myBelt.setBeltPosition(beltPosition);
        }
    }
}
