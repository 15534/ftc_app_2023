package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Belt {
    public Servo beltServo;

    public int drift = 0;
    ElapsedTime timer = new ElapsedTime();
    public double moveTime = 2.0;
    public double updateTime = 0.01;
    double numUpdates = moveTime/updateTime;
    double targetPos, updates;

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

    public void slowMove(Consts.Belt target) {
        switch (target) {
            case UP:
                targetPos = (Consts.BELT_UP_LIMIT);
                break;
            case DOWN:
                targetPos = (Consts.BELT_DOWN_LIMIT);
                break;
            case CONE_DROP:
                targetPos = (Consts.BELT_DROP_LIMIT);
                break;
        }
    }

    public void slowMoveUpdate(){
        if (0.05 > Math.abs(targetPos-beltServo.getPosition())){
            beltServo.getPosition();
        }
        else {
            if (updates < numUpdates) {
                if (timer.time() == updateTime){
                    double newTarget = targetPos - updates*(targetPos/numUpdates);
                    beltServo.setPosition(newTarget);
                    updates++;
                    timer.reset();
                }
            }
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
