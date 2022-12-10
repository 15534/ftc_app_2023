package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "LiftLimits")
@Config()

public class LiftLimits extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "rightLift");

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(0.1);
        right.setPower(0.1);

        left.setTargetPosition(0);
        right.setTargetPosition(0);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Left: ", left.getCurrentPosition());
            telemetry.addData("Right: ", right.getCurrentPosition());

            telemetry.update();
        }
    }
}
