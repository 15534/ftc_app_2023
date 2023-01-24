package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.CLAW_CLOSE_LIMIT;
import static org.firstinspires.ftc.teamcode.subsystems.Consts.CLAW_OPEN_LIMIT;

public class Claw {
    public Servo clawServo;

    public void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        move(Consts.Claw.CLOSECLAW);
    }

    public void move(Consts.Claw input) {
        // Servo limits go from 0 to 1
        switch (input) {
            case OPENCLAW:
                clawServo.setPosition(CLAW_OPEN_LIMIT);
                break;
            case CLOSECLAW:
                clawServo.setPosition(CLAW_CLOSE_LIMIT);
                break;
        }
    }

    public void moveClaw(double target) {
        clawServo.setPosition(target);
    }

    public void reset() {
        move(Consts.Claw.CLOSECLAW);
    }

    public double getPosition() {
        return clawServo.getPosition();
    }
}
