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
                moveAbsolute(Consts.BELT_UP_LIMIT);
                break;
            case DOWN:
                moveAbsolute(Consts.BELT_DOWN_LIMIT);
                break;
            case CONE_DROP:
                moveAbsolute(Consts.BELT_DROP_LIMIT);
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
