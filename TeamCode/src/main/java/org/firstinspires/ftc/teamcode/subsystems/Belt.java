package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Belt {
    public DcMotorEx belt;

    public static double targetPos = 0;
    private final double gain = -.001;

    public void init(HardwareMap hardwareMap) {
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        moveBelt(Constants.IntakeTargets.HOLD);
    }

    public void moveBelt(Constants.IntakeTargets input) {
        switch (input) {
            case PICKUP:
                setBeltPosition(-261);
                break;
            case HOLD:
                setBeltPosition(0);
                break;
            case DROPOFF:
                setBeltPosition(-281);
                break;
        }
    }

    public void setBeltPosition(double targetPosition) {
        targetPos = targetPosition;
    }

    public void updateBeltPosition() {
        if (4 > Math.abs(targetPos - belt.getCurrentPosition())) {
            belt.getCurrentPosition();
        } else {
            double newPower = (targetPos - belt.getCurrentPosition()) * gain;
            belt.setPower(newPower);
        }
    }

    public int getPosition() {
        return belt.getCurrentPosition();
    }
}
