//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.subsystems.LiftPID;
//
//@TeleOp(name = "LiftLimits")
//@Config()
//public class LiftLimits extends LinearOpMode {
//
//    public static double leftPowerInitial = 0.1; // counterclockwise
//    public static double rightPowerInitial = -0.1; // clockwise
//
//    public static double leftPos = 20; // counterclockwise
//    public static double rightPos = -20; // clockwise
//
//    private LiftPID lift = new LiftPID();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        waitForStart();
//
//        lift.init(hardwareMap);
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//
//            telemetry.addData("Left: ", left.getCurrentPosition());
//            telemetry.addData("Right: ", right.getCurrentPosition());
//
//            telemetry.update();
//        }
//    }
//}
