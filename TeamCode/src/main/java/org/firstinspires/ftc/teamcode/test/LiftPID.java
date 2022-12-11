package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "LiftPID")
@Config()
public class LiftPID extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "rightLift");

        left.setPower(0.1);
        right.setPower(0.1);

        //        left.setTargetPosition();
        //        right.setTargetPosition();

    }
}
