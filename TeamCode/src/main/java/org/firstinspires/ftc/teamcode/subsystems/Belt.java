package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Belt {
    public Servo beltServo;

    public int drift = 0;
    ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        beltServo = hardwareMap.get(Servo.class, "beltServo");
        timer.reset();
    }

    public void move(Consts.Belt target) {
        switch (target) {
            case UP:
                moveAbsolute(0.75);
                break;
            case DOWN:
                moveAbsolute(0.20);
                break;
            case CONE_DROP:
                moveAbsolute(0.25);
                break;
        }
    }

    public void moveAbsolute(double target){
        beltServo.setPosition(target);
    }


    public void reset() {
        move(Consts.Belt.UP);
    }

    public double getPosition() {
        return beltServo.getPosition();
    }
}
