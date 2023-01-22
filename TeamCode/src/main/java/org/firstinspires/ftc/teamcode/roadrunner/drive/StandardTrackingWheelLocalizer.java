package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1440; // Encoder used: S4T Miniature Optical Shaft Encoder
    // hypothesis: CPR * 4 = PPR = TICKS_PER_REV, thus CPR = 360

    // Data below was retrieved from 2021 code
    public static double WHEEL_RADIUS = 1.147; // in, from 2021 codebase
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    // The back encoder is geared differently (40:24)
    public static double GEAR_RATIO_BACK = 40.0/24; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 9.6; // in; distance between the left and right wheels
    // FORWARD_OFFSET is negative because it is behind the lateral encoders
    public static double FORWARD_OFFSET = -6.73; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, backEncoder;

    public static double X_MULTIPLIER = 100/100.5625366389; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 100/98.9672081053; // Multiplier in the Y direction

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));

        // Reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        // Reached this conclusion through trial and error
        // Byran 12.20.22
        backEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    // backEncoder has a different gear ratio (40:24), from 2021 codebase
    // Byran 12.20.22
    public static double encoderTicksToInchesBack(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV / (GEAR_RATIO_BACK);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInchesBack(backEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getRawVelocity() * X_MULTIPLIER),
                encoderTicksToInches(backEncoder.getRawVelocity() * Y_MULTIPLIER)
        );
    }
}
