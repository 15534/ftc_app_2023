package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class OpenCV {
    public static class Pipeline extends OpenCvPipeline {
        public enum Orientation {
            CYAN,
            MAGENTA,
            YELLOW,
            PROCESSING
        }

        static final Scalar CYAN = new Scalar(0, 255, 255);
        static final Scalar MAGENTA = new Scalar(255, 0, 255);
        static final Scalar YELLOW = new Scalar(255, 255, 0);

        public static final Point REGION_A = new Point(150, 90);
        public static final Point REGION_B = new Point(170, 120);

        public static int avg1, avg2, avg3;

        Mat submat;
        static int red;
        static int blue;
        static int green;

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Orientation position = Orientation.PROCESSING;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(REGION_A, REGION_B));
            submat = firstFrame.submat(new Rect(REGION_A, REGION_B));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            red = (int) Core.mean(submat).val[0];
            green = (int) Core.mean(submat).val[1];
            blue = (int) Core.mean(submat).val[2];

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region1_Cb).val[1];
            avg3 = (int) Core.mean(region1_Cb).val[2];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_A, // First point which defines the rectangle
                    REGION_B, // Second point which defines the rectangle
                    CYAN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            return input;
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public Orientation getAnalysis() {
            return position;
        }

        public static int getRed() {
            return red;
        }

        public static int getBlue() {
            return blue;
        }

        public static int getGreen() {
            return green;
        }

        public static int getAvg1() {
            return avg1;
        }

        public static int getAvg2() {
            return avg2;
        }

        public static int getAvg3() {
            return avg3;
        }

        public static String getColor() {

            // Cyan (0, 255, 255)
            if (red < 90 && blue < 90 && green < 90) {
                return "Black";
            }

            if (blue > red && blue > green) {
                return "Blue";
            }

            return "Yellow";
        }
    }
}
