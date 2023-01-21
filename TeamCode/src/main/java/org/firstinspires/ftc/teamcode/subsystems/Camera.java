package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.OpenCV;
import org.openftc.easyopencv.*;


public class Camera extends LinearOpMode {

    OpenCvWebcam webcam;
    OpenCV.Pipeline pipeline;
    OpenCV.Pipeline.Orientation snapshotAnalysis = OpenCV.Pipeline.Orientation.PROCESSING; // default

    public void init(HardwareMap hardwareMap) {
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
        pipeline = new OpenCV.Pipeline();
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
                        telemetry.addData("Camera Initialization: ", "Failed");
                        telemetry.update();
                    }
                });

        snapshotAnalysis = pipeline.getAnalysis();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}
