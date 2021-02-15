package org.firstinspires.ftc.teamcode.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.util.Encoder.Direction;

public class RROdometryLocalizer extends ThreeTrackingWheelLocalizer {

  // E8T-360-250-...
  public static double TICKS_PER_REV = 360;
  public static double WHEEL_RADIUS = 38.0 / 2 / 25.4; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

  private final Encoder leftEncoder, rightEncoder, centerEncoder;

  public RROdometryLocalizer(HardwareMap hardwareMap) {
    // First calculated in https://docs.google.com/document/d/1s6HzvajxItlIaULulVud0IRhnVrH16yjvsGW4jbwosQ/edit
    super(Arrays.asList(
        new Pose2d(-0.5485, 7.7588, 0), // left (no electronics)
        new Pose2d(-0.5485, -7.7588, 0), // right (with electronics)
        // TODO change this if they don't correspond in position
        new Pose2d(-4.9242, 0.5485, Math.toRadians(90)) // center
    ));

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
    centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "centerEncoder"));

    // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    rightEncoder.setDirection(Direction.REVERSE);
  }

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() {
    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getCurrentPosition()),
        encoderTicksToInches(rightEncoder.getCurrentPosition()),
        encoderTicksToInches(centerEncoder.getCurrentPosition())
    );
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
    //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
    //  compensation method

    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getRawVelocity()),
        encoderTicksToInches(rightEncoder.getRawVelocity()),
        encoderTicksToInches(centerEncoder.getRawVelocity())
    );
  }
}
