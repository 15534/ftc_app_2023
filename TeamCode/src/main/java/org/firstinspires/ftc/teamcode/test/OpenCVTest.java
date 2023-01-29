package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.OpenCV;
import org.openftc.easyopencv.*;

import java.util.ArrayList;

@Autonomous(name = "OpenCVTest")
public class OpenCVTest extends LinearOpMode {

    OpenCvCamera webcam;
    OpenCV.Pipeline pipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        pipeline = new OpenCV.Pipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(pipeline);

        ((OpenCvWebcam) webcam).setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                        webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        telemetry.addData("Camera Initialization: ", errorCode);
                        telemetry.update();
                    }
                });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();
        while (opModeIsActive()) {

            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    telemetry.addData("Tag ID ", tag.id);
                }

            }
            else
            {
                telemetry.addData("Tag ID ", -1);
            }

            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }
}
