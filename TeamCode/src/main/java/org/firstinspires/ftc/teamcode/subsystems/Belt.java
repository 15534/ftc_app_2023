package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Belt {
    public DcMotorEx belt;

    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
    }

    public void moveBelt(Constants.IntakeTargets target) {
        switch (target) {
            case PICKUP:
                belt.setTargetPosition(0);
                break;
                //            case HOLD:
                //                setBeltPosition(0);
                //                break;
            case DROPOFF:
                belt.setTargetPosition(-269);
                break;
        }

        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt.setPower(0.5);
    }
}
