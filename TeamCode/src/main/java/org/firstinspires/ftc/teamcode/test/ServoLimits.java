package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "ServoLimits")
@Config

public class ServoLimits extends LinearOpMode{
    public static double limitOne = 0.992;
    public static double limitTwo = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo one = hardwareMap.get(Servo.class, "claw");


        waitForStart();
        while (opModeIsActive()) {

            // one.setPosition(limitOne);
            telemetry.addData("controller: ", one.getController());

            one.setPosition(1);
            sleep(2000);
            one.setPosition(0);
            sleep(2000);

            telemetry.addData("direction: ", one.getDirection());
            telemetry.addData("position: ", one.getPosition());
            telemetry.update();
        }
    }
}
