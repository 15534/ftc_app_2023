package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurnTable {
    private DcMotorEx motor;

    double INIT_MOTOR_POWER = 0.1;

    public TurnTable(HardwareMap hardwareMap) {
        // @TODO: FIX NAME
        motor = hardwareMap.get(DcMotorEx.class, "TurntableDriving");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void init() {
        motor.setPower(INIT_MOTOR_POWER);
        motor.setTargetPosition(0);
    }

    public void turn(int degrees) {
        double rad = Math.toRadians(degrees);

        // @TODO: Find motor turn range using getCurrentPositions()
        motor.setTargetPosition(0);
    }
}
