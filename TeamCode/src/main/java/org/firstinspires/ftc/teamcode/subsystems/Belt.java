package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Belt {
    public DcMotorEx belt;
    double encoderPos = 0;
    double previousEncoderPos = 0;
    boolean goBackCalled = false;
    public int drift = 0;
    ElapsedTime timer = new ElapsedTime();


    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveBelt(Constants.IntakeTargets target) {
        switch (target) {
            case PICKUP:
                belt.setTargetPosition(drift);
                belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                belt.setPower(.5);
                break;
            case DROPOFF:
                drift = belt.getCurrentPosition();
                belt.setTargetPosition(-285 + drift);
                belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                belt.setPower(0.5);
                break;
        }
    }

    public void moveBeltAbsolute(int target){
        belt.setTargetPosition(target);
        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        belt.setPower(0.5);
    }
}
