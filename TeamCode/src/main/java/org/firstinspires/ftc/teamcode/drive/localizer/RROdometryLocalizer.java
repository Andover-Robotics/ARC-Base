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

  // This is rev through bore encoder now
  public static double TICKS_PER_REV = 8192;
  public static double WHEEL_RADIUS = 38.75 / 2 / 25.4; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
  public static double CENTER_MULT = 24 / 23.55;

  private final Encoder leftEncoder, rightEncoder, centerEncoder;

  public RROdometryLocalizer(HardwareMap hardwareMap) {
    this(hardwareMap, new Pose2d(-0.55, 7.3, 0), new Pose2d(5.6, 0.512, Math.toRadians(90)));
  }

  public RROdometryLocalizer(HardwareMap hardwareMap, Pose2d sidePose, Pose2d centerPose) {
    // First calculated in https://docs.google.com/document/d/1s6HzvajxItlIaULulVud0IRhnVrH16yjvsGW4jbwosQ/edit
    super(Arrays.asList(
        new Pose2d(-sidePose.getX(), sidePose.getY(), 0), // left (no electronics)
        new Pose2d(-sidePose.getX(), -sidePose.getY(), 0), // right (with electronics)
        // TODO change this if they don't correspond in position
        centerPose // center
    ));

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
    centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFL"));

    leftEncoder.setDirection(Direction.FORWARD);
    rightEncoder.setDirection(Direction.FORWARD);
    centerEncoder.setDirection(Direction.REVERSE);
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
        encoderTicksToInches(CENTER_MULT * centerEncoder.getCurrentPosition())
    );
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
        encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
        encoderTicksToInches(CENTER_MULT * centerEncoder.getCorrectedVelocity())
    );
  }
}
