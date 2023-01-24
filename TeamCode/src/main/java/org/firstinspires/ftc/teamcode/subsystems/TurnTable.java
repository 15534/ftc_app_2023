package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// NOTE: Turntable must be initialized facing forward
public class TurnTable {
    DcMotorEx motor;

    // 90 degrees: -925
    // 180: -1863
    // 270: -2805
    double TICKS_PER_DEGREE = -10.3698;

    public static double MOTOR_POWER = 1;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turnTable");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double degrees) {
        int target = (int) degrees;

        if (target >= 270) {
            target = 270;
        }

        if (target <= -270) {
            target = -270;
        }

        motor.setTargetPosition((int) (target * TICKS_PER_DEGREE));
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(MOTOR_POWER);
    }

    public void reset() {
        move(0);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }
}
