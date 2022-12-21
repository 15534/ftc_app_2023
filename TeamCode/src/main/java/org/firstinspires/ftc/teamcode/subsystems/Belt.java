package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Belt {
    private DcMotorEx belt;

    public static double BELT_POWER = 0.05;

    public Belt(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "belt");
    }

    public void init(HardwareMap hardwareMap) {
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void goUp(){
        belt.setTargetPosition(-140);  //-280
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belt.setPower(BELT_POWER);
    }

    public void moveBelt(int position) {
        belt.setTargetPosition((int) position);
        belt.setPower(BELT_POWER);
    }
}
