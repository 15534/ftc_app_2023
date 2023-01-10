package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config()
public class PID {
//    For the up movement
//    public static double Kp = 0.1;
//    public static double Ki = 0.05;
//    public static double Kd = .0008;

//    For the down movement
//    public static double Kp = .05;
//    public static double Ki = 0;
//    public static double Kd = .001;

//    for upwards new

    public static double Kp = .15;
    public static double Ki = .07;
    public static double Kd = .001;


    double target = 69;
    double error;
    double derivative;
    double lastError;

    double integralSum = 0;

    double out;
    ElapsedTime timer;

    public void init(ElapsedTime newTimer){
        timer = newTimer;
        lastError = 0;
    }

    public void setTarget(double newTarget){
        target = newTarget;
    }

    public double update(int motorPosition){
        error = target - motorPosition;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        timer.reset();

        return out;
    }
}
