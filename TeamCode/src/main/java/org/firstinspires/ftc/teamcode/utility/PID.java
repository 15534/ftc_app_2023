package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config()
public class PID {
    public static double Kp = 0.4;
    public static double Ki = 0; //0.05
    public static double Kd = 0; //1.02
    double target = 0;
    double error;
    double derivative;
    double lastError;
    double integralSum = 0;
    double out;
    ElapsedTime timer;

    public void init(ElapsedTime newTimer) {
        timer = newTimer;
        lastError = 0;
    }

    public void init(ElapsedTime newTimer, double newp, double newi, double newd) {
        Kp = newp;
        Ki = newi;
        Kd = newd;
        timer = newTimer;
        lastError = 0;
    }

    public void setTarget(double newTarget) {
        target = newTarget;
    }

    public double update(int motorPosition) {
        error = target - motorPosition;
        derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        lastError = error;
        timer.reset();
        return out;
    }
}
