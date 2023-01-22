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

@Config()
public class OpenCV
{
    public static class Pipeline extends OpenCvPipeline
    {
        public enum Orientation
        {
            CYAN,
            MAGENTA,
            YELLOW,
            PROCESSING
        }

        static final Scalar CYAN = new Scalar(0, 255, 255);
        static final Scalar MAGENTA = new Scalar(255, 0, 255);
        static final Scalar YELLOW = new Scalar(255, 255, 0);

        public static final Point REGION_A = new Point(160, 120);
        public static final Point REGION_B = new Point(180, 150);

        Mat submat;
        int red, blue, green;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Orientation position = Orientation.PROCESSING;

        @Override
        public void init(Mat firstFrame)
        {
            submat = firstFrame.submat(160,180,120,180);
        }

        @Override
        public Mat processFrame(Mat input)
        {
        // TESTED
            red = (int) Core.mean(submat).val[0];
            blue = (int) Core.mean(submat).val[1];
            green = (int) Core.mean(submat).val[2];


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_A, // First point which defines the rectangle
                    REGION_B, // Second point which defines the rectangle
                    CYAN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            return input;
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public Orientation getAnalysis()
        {
            return position;
        }

        public int getRed(){
            return red;
        }

        public int getBlue(){
            return blue;
        }

        public int getGreen(){
            return green;
        }

        public String getColor(){

            // Cyan (0, 255, 255)
            if (red < 20 && blue > 225 && green > 225) {
                return "Cyan";
            }

            // Magenta (255, 0, 255)
            if (red > 225 && blue < 20 && green > 225)  {
                return "Magenta";
            }

            // Yellow (255, 255, 0)
            if (red > 225 && blue > 225 && green < 20) {
                return "Yellow";
            }

            return "Nothing";
        }
    }
}