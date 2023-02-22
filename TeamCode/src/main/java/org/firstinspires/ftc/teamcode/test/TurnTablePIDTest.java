package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.subsystems.TurnTable;
import org.firstinspires.ftc.teamcode.utility.PID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TurnTablePIDTest")
@Config
public class TurnTablePIDTest extends LinearOpMode {

    public static double degreePosition = 180;
    double TICKS_PER_DEGREE = -10.3698;

    @Override
    public void runOpMode() throws InterruptedException {
        TurnTable turntable = new TurnTable();
        PID turnTablePID = new PID();
        ElapsedTime ttTimer = new ElapsedTime();
        ElapsedTime mainTimer = new ElapsedTime();

        turnTablePID.init(ttTimer);
        turntable.init(hardwareMap);

        waitForStart();
        mainTimer.reset();


        turnTablePID.setTarget(degreePosition * TICKS_PER_DEGREE);
        turntable.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(turntable.motor.getCurrentPosition()/TICKS_PER_DEGREE - degreePosition) > .5) {
            turntable.motor.setPower(turnTablePID.update(turntable.getPosition()));
        }
        telemetry.addData("time", mainTimer.milliseconds());
        telemetry.update();
        while (opModeIsActive()){
            turntable.getPosition();
        }

    }
}
