package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.clawCloseLimit;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.clawOpenLimit;

public class Claw {

//    public static double beltPower = 0.05;

    private Servo clawServo;

    public void init(HardwareMap hardwareMap) {
        clawServo =  hardwareMap.get(Servo.class, "claw");
        moveClaw(Constants.ClawTargets.CLOSECLAW);
    }

    public void moveClaw(Constants.ClawTargets input) {
        // limits are 0 and 1
        switch (input) {
            case OPENCLAW:
                clawServo.setPosition(clawOpenLimit);
                break;
            case CLOSECLAW:
                clawServo.setPosition(clawCloseLimit);
                break;
        }
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }


}
