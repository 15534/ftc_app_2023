package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Belt {
    public DcMotorEx belt;

    public static double targetPosition = 0;
    private final double gain = -.001;
    double beltUpPos = Constants.BELT_UP_POSITION;
    double beltDownPos = Constants.BELT_DOWN_POSITION;
    double encoderPos = 0;
    double previousEncoderPos = 0;
    boolean goBackCalled = true;

    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        moveBelt(Constants.IntakeTargets.HOLD);
    }

    public void moveBelt(Constants.IntakeTargets input) {
        switch (input) {
            case PICKUP:
                setBeltPosition(-10);
                break;
//            case HOLD:
//                setBeltPosition(0);
//                break;
            case DROPOFF:
                setBeltPosition(-269);
                break;
        }
    }

    public void setBeltPosition(double newTarget) {
        targetPosition = newTarget;
    }

    public void updateBeltPosition() {

        if (targetPosition == 0 && goBackCalled){
            belt.setPower(-.5);
            goBackCalled = false;
        }
        else if (targetPosition == 0){
            previousEncoderPos = encoderPos;
            encoderPos = belt.getCurrentPosition();
            if (encoderPos == previousEncoderPos && previousEncoderPos != 0){
                belt.setPower(0);
                belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                goBackCalled = true;
            }
        }
        else{
            if (4 > Math.abs(targetPosition - belt.getCurrentPosition())) {
                belt.setPower(0);

            } else {
                double newPower = (targetPosition - belt.getCurrentPosition()) * gain;
                belt.setPower(newPower);
            }
        }




    }

    public int getPosition() {
        return belt.getCurrentPosition();
    }
}
