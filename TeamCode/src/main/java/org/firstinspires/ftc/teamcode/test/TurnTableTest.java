package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.subsystems.TurnTable;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TurnTableTest")
@Config
public class TurnTableTest extends LinearOpMode {

    public static double degreePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TurnTable turntable = new TurnTable();
        turntable.init(hardwareMap);



        waitForStart();

        while (opModeIsActive()) {
            turntable.turn(degreePosition);

            telemetry.addData("Turntable Position: ", turntable.getCurrentPosition());
            telemetry.update();
        }
    }
}
