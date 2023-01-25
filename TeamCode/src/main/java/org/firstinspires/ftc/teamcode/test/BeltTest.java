//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.Belt;
//
//@TeleOp(name = "BeltTest")
//@Config
//public class BeltTest extends LinearOpMode {
//    public static int beltPosition = 0;
//
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry dashboardTelemetry = dashboard.getTelemetry();
//
//    public Belt belt = new Belt();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
//
//        belt.init(hardwareMap);
//
//        waitForStart();
//        while (opModeIsActive() && !isStopRequested()) {
//            belt.moveNoCorrection(beltPosition);
//
//            telemetry.addData("Belt pos", belt.belt.getCurrentPosition());
//            telemetry.addData("Belt power", belt.belt.getPower());
//            telemetry.addData("Belt target", belt.belt.getTargetPosition());
//            telemetry.update();
//        }
//    }
//}
