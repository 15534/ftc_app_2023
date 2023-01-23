package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Belt {
    public DcMotorEx belt;
    public int drift = 0;
    ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        timer.reset();
        belt = hardwareMap.get(DcMotorEx.class, "intakeLift");
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move (Consts.Belt target) {
            switch (target) {
                case UP:
                    belt.setTargetPosition(drift);
                    belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    belt.setPower(.5);
                    break;
                case DOWN:
                    drift = belt.getCurrentPosition();
                    belt.setTargetPosition(-280 + drift);
                    belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    belt.setPower(0.5);
                    break;
                case CONE_DROP:
                    belt.setTargetPosition(-274+drift);
                    belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    belt.setPower(0.5);
                    break;
        }
    }

    public void moveNoCorrection(Consts.Belt target) {
        switch (target) {
            case UP:
                belt.setTargetPosition(0);
                belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                belt.setPower(0.5);
                break;
            case DOWN:
                belt.setTargetPosition(-280);
                belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                belt.setPower(0.5);
                break;
            case CONE_DROP:
                belt.setTargetPosition(-274);
                belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                belt.setPower(0.5);
                break;
        }
    }

    // Raw integer input
    public void moveNoCorrection(int target) {
        belt.setTargetPosition(target);
        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt.setPower(0.5);
    }

    public void reset() {
        move(Consts.Belt.UP);
    }

    public int getPosition() {
        return belt.getCurrentPosition();
    }
}
