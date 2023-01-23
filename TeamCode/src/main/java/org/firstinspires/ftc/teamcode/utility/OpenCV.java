package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class OpenCV {
    public static class Pipeline extends OpenCvPipeline {

        public enum Orientation {
            PROCESSING
        }

        static final Scalar YELLOW = new Scalar(255, 255, 0);

        public static final Point REGION_A = new Point(50, 50);   // x and y are flipped because of vertical camera mount
        public static final Point REGION_B = new Point(550, 1200);

        Mat submatrix;
        private static String data, dataCurved;
        QRCodeDetector qrDecoder = new QRCodeDetector();

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Orientation position = Orientation.PROCESSING;

        @Override
        public void init(Mat firstFrame) {
            submatrix = firstFrame.submat(new Rect(REGION_A, REGION_B));
        }

        @Override
        public Mat processFrame(Mat input) {

            data = qrDecoder.detectAndDecode(submatrix);
            dataCurved = qrDecoder.detectAndDecodeCurved(submatrix);

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_A, // First point which defines the rectangle
                    REGION_B, // Second point which defines the rectangle
                    YELLOW, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            return input;
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public Orientation getAnalysis() {
            return position;
        }

        public static String getQR(){
            return data;
        }

        public static String getQRCurved(){
            return dataCurved;
        }
    }
}
