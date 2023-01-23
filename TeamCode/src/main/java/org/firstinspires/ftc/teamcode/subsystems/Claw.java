package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.clawCloseLimit;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.clawOpenLimit;

public class Claw {
    public Servo clawServo;

    public void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        moveClaw(Constants.ClawTargets.CLOSECLAW);
    }

    public void moveClaw(Constants.ClawTargets input) {
        // Servo limits go from 0 to 1
        switch (input) {
            case OPENCLAW:
                clawServo.setPosition(clawOpenLimit);
                break;
            case CLOSECLAW:
                clawServo.setPosition(clawCloseLimit);
                break;
        }
    }

    // Raw integer input
    public void moveClaw(double target) {
        clawServo.setPosition(target);
    }
    
    public double getPosition() {
        return clawServo.getPosition();
    }
}
