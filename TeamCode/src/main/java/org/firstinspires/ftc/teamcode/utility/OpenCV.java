package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class OpenCV {
    public static class Pipeline extends OpenCvPipeline {

        private long nativeApriltagPtr;
        private Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();

        Mat cameraMatrix;

        public static final Point REGION_A = new Point(0, 30);
        public static final Point REGION_B = new Point(360, 690);

        double fx;
        double fy;
        double cx;
        double cy;

        // UNITS ARE METERS
        double tagsize;
        double tagsizeX;
        double tagsizeY;

        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();

        Mat region1;

        @Override
        public void init(Mat firstFrame) {
            region1 = firstFrame.submat(new Rect(REGION_A, REGION_B));
        }

        public Pipeline(double tagsize, double fx, double fy, double cx, double cy)
        {
            this.tagsize = tagsize;
            this.tagsizeX = tagsize;
            this.tagsizeY = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            constructMatrix();

            // Allocate a native context object. See the corresponding deletion in the finalizer
            nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        public void finalize()
        {
            // Might be null if createApriltagDetector() threw an exception
            if(nativeApriltagPtr != 0)
            {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                nativeApriltagPtr = 0;
            }
            else
            {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }

        @Override
        public Mat processFrame(Mat input) {

            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync)
            {
                if(needToSetDecimation)
                {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync)
            {
                detectionsUpdate = detections;
            }

            return input;
        }

        public void setDecimation(float decimation)
        {
            synchronized (decimationSync)
            {
                this.decimation = decimation;
                needToSetDecimation = true;
            }
        }

        public ArrayList<AprilTagDetection> getLatestDetections()
        {
            return detections;
        }

        public ArrayList<AprilTagDetection> getDetectionsUpdate()
        {
            synchronized (detectionsUpdateSync)
            {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }

        void constructMatrix() {
            //     Construct the camera matrix.
            //
            //      --         --
            //     | fx   0   cx |
            //     | 0    fy  cy |
            //     | 0    0   1  |
            //      --         --
            //

            cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

            cameraMatrix.put(0,0, fx);
            cameraMatrix.put(0,1,0);
            cameraMatrix.put(0,2, cx);

            cameraMatrix.put(1,0,0);
            cameraMatrix.put(1,1,fy);
            cameraMatrix.put(1,2,cy);

            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2,1,0);
            cameraMatrix.put(2,2,1);
        }
    }
}
