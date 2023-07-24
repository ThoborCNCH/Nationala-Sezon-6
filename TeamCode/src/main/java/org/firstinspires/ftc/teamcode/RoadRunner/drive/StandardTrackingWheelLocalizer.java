package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;

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
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.905512; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    //12.23472
    //12.305
    public static double LATERAL_DISTANCE = 13.567; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4.82748; // in; offset of the lateral wheel

//5.254961
//    1.081851970459335
//    1.084624294812401
//    public static double X_MULTIPLIER = 1.094016680750096; // Multiplier in the X direction
//    public static double Y_MULTIPLIER = 1.084624294812401; // Multiplier in the Y direction


    public static double X_MULTIPLIER = 1.436060189240141; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.452631591608717; // Multiplier in the Y direction


    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "brat"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = (int) (leftEncoder.getCurrentPosition() * X_MULTIPLIER);
        int rightPos = (int) (rightEncoder.getCurrentPosition() * X_MULTIPLIER);
        int frontPos = (int) (frontEncoder.getCurrentPosition() * Y_MULTIPLIER);

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) (leftEncoder.getCorrectedVelocity() * X_MULTIPLIER);
        int rightVel = (int) (rightEncoder.getCorrectedVelocity() * X_MULTIPLIER);
        int frontVel = (int) (frontEncoder.getCorrectedVelocity() * Y_MULTIPLIER);

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }
}
