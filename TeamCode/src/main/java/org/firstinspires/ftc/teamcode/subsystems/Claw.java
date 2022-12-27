package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    public static double beltPower = 0.05;

    private DcMotorEx intakeLift;
    private Servo clawServo;

    public Claw(HardwareMap hardwareMap) {
        clawServo =  hardwareMap.get(Servo.class, "claw");
    }

    public void moveClaw(Constants.ClawTargets input) {
        // limits are 0 and 1
        switch (input) {
            case OPENCLAW:
                clawServo.setPosition(0);
                break;
            case CLOSECLAW:
                clawServo.setPosition(1);
                break;
        }
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }
}
