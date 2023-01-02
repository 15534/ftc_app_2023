package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Belt {
    public DcMotorEx belt;

    public static double BELT_POWER = 0.001;
    public static double targetPos = 0;
    private double gain = -.01;

    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        belt.setDirection(DcMotorEx.Direction.REVERSE);
        moveBelt(Constants.IntakeTargets.HOLD);
    }

    public void moveBelt(Constants.IntakeTargets input) {
        switch (input) {
            case PICKUP:
                setBeltPosition(-261);
                updateBeltPosition();
                break;
            case HOLD:
                setBeltPosition(0);
                updateBeltPosition();
                break;
            case DROPOFF:
                setBeltPosition(-285);
                updateBeltPosition();
                break;
        }
    }

    public void setBeltPosition(double targetPosition){
        targetPos = targetPosition;
//        belt.getCurrentPosition();
    }

    public void updateBeltPosition(){
        if (4>Math.abs(targetPos-belt.getCurrentPosition())){
            belt.getCurrentPosition();
        }
        else{
            double newPower = (targetPos-belt.getCurrentPosition())*gain;
            belt.setPower(newPower);
        }
    }

    public int getPosition() {
        return belt.getCurrentPosition();
    }
}
