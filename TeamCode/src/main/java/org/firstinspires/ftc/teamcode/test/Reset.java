package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Belt;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TurnTable;

@TeleOp(name = "Reset")
@Config

public class Reset extends LinearOpMode {
    public static int beltPosition = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Belt belt = new Belt();
    public Claw claw = new Claw();
    public Lift lift = new Lift();
    public TurnTable turnTable = new TurnTable();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        belt.init(hardwareMap);
        claw.init(hardwareMap);
        lift.init(hardwareMap);
        turnTable.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            belt.reset();
            claw.reset();
            lift.reset();
            turnTable.reset();
        }
    }
}
