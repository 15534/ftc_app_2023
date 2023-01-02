package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurnTable {
    DcMotorEx motor;

    double TICKS_PER_DEGREE = -27.7593;
    int OFFSET = -50;

    public static double MOTOR_POWER = 1;

    // Our data: Telemetry: Degrees
    // -200 : -50 : 0 deg
    // -10200 : -2576 : 90 deg
    // -19975 : -5045 : 180 deg
    // 9600 : 2424 : -90 deg

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turnTable");
        motor.setTargetPosition(0); // mx + b (setTarget Position accepts motor ticks)
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(MOTOR_POWER);
    }

    public void turn(double degrees) {
        motor.setTargetPosition((int) (degrees * TICKS_PER_DEGREE) + OFFSET); // mx + b (setTarget Position accepts motor ticks)
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(MOTOR_POWER);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
