package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// NOTE: Turntable must be initialized facing forward
public class TurnTable {
    DcMotorEx motor;

    double TICKS_PER_DEGREE = -27.7593;

    public static double MOTOR_POWER = 1;

    public static double targetPos = 0;
    private double gain = -.01;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turnTable");
        setTablePosition(0);
    }

    public void setTablePosition(double degrees) {
        targetPos = (int) (degrees * TICKS_PER_DEGREE);
    }

    public void updateTablePosition() { // set a safety for turntable later
        if (Math.abs(targetPos - motor.getCurrentPosition()) > 4) {
            double newPower = (motor.getCurrentPosition() - targetPos) * gain;
            motor.setPower(newPower);
        }
    }

    // @TODO: Verify deprecation
    // TO-DO: don't use turn - it is deprecated (remove in future from AsyncTest & TurnTableTest)
    public void turn(double degrees) {
        motor.setTargetPosition(
                (int) (degrees * TICKS_PER_DEGREE)); // mx + b (setTarget Position accepts motor
        // ticks)
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motor.setPower(MOTOR_POWER);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
