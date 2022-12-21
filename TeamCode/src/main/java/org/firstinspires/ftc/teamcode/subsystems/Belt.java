package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Belt {
    public DcMotorEx belt;

    public static double BELT_POWER = 0.001;

    public Belt() {
    }

    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void goUp(){
        belt.setTargetPosition(-140);  //-280
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belt.setPower(BELT_POWER);
    }

    public void moveBelt(int position) {
        belt.setTargetPosition((int) position);
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belt.setPower(BELT_POWER);
    }

    public int getPosition() {
        return belt.getCurrentPosition();
    }
}
