package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@TeleOp(name = "TurnTableTest")
@Config
public class TurnTableTest extends LinearOpMode {

    public static double degreePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TurnTable turntable = new TurnTable();
        turntable.init(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        turntable.move(degreePosition);
        while (turntable.motor.isBusy()) {
            telemetry.addData("motorPos", turntable.motor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("time", timer.seconds());
        telemetry.update();
        while (opModeIsActive()){
            turntable.getPosition();
        }

    }
}
