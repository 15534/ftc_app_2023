package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Belt {
    public DcMotorEx belt;

    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
    }

    public void moveBelt(int target) {
        belt.setTargetPosition(target);

        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt.setPower(0.5);
    }
}
