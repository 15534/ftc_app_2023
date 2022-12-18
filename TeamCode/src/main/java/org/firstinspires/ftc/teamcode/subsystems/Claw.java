package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    public static double beltPower = 0.05;

    private DcMotorEx intakeLift;
    private Servo clawServo;

    public void init(HardwareMap hardwareMap) {
        intakeLift = hardwareMap.get(DcMotorEx.class, "intakeLift");
        intakeLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        clawServo =  hardwareMap.get(Servo.class, "claw");

        moveBelt(Constants.IntakeTargets.HOLD);
        moveClaw(Constants.ClawTargets.CLOSECLAW);

    }

    public void pickUpCone() throws InterruptedException {
        moveBelt(Constants.IntakeTargets.PICKUP);
        moveClaw(Constants.ClawTargets.OPENCLAW);

        sleep(2000);
        moveClaw(Constants.ClawTargets.CLOSECLAW);
        sleep(2000);
        moveBelt(Constants.IntakeTargets.HOLD);

    }

    public void dropOffCone() throws InterruptedException {
        moveBelt(Constants.IntakeTargets.DROPOFF);
        moveClaw(Constants.ClawTargets.OPENCLAW);
    }

    public void moveBelt(int position) {
        intakeLift.setTargetPosition((int) position);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLift.setPower(beltPower);
    }

    public void moveBelt(Constants.IntakeTargets input) {
        switch (input) {
            case PICKUP:
                moveBelt(-500);
                break;
            case HOLD: // init
                moveBelt(-262);
                break;
            case DROPOFF:
                moveBelt(-554);
                break;
        }
    }

    public void moveClaw(Constants.ClawTargets input) {
        // limits are 0 and 1
        switch (input) {
            case OPENCLAW:
                clawServo.setPosition(0);
                break;
            case CLOSECLAW:
                clawServo.setPosition(1);
                break;
        }
    }
}
