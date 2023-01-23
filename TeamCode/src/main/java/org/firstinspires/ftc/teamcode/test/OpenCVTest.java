package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.OpenCV;
import org.openftc.easyopencv.*;

@Autonomous(name = "OpenCVTest")
public class OpenCVTest extends LinearOpMode {

    OpenCvWebcam webcam;
    OpenCV.Pipeline pipeline;
    OpenCV.Pipeline.Orientation snapshotAnalysis =
            OpenCV.Pipeline.Orientation.PROCESSING; // default

    @Override
    public void runOpMode() {

        // Telemetry Initialization
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Camera Initialization
        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier(
                                "cameraMonitorViewId",
                                "id",
                                hardwareMap.appContext.getPackageName());
        webcam =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(
                                hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        pipeline = new OpenCV.Pipeline();
        webcam.setPipeline(pipeline);

        ((OpenCvWebcam) webcam).setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        telemetry.addData("Camera Initialization: ", errorCode);
                        telemetry.update();
                    }
                });

        snapshotAnalysis = pipeline.getAnalysis();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("anal", pipeline.getAnalysis());
            telemetry.addData("red", OpenCV.Pipeline.getRed());
            telemetry.addData("blue", OpenCV.Pipeline.getBlue());
            telemetry.addData("green", OpenCV.Pipeline.getGreen());
            telemetry.addData("color", OpenCV.Pipeline.getColor());
            telemetry.addData("avg 1", OpenCV.Pipeline.getAvg1());
            telemetry.addData("avg 2", OpenCV.Pipeline.getAvg2());
            telemetry.addData("avg 3", OpenCV.Pipeline.getAvg3());

            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }
}
