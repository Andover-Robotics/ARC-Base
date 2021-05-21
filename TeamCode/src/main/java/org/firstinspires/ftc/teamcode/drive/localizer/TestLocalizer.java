package org.firstinspires.ftc.teamcode.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Encoder.Direction;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import java.util.Arrays;
import java.util.Objects;

public class TestLocalizer implements Localizer {
  public Localizer[] localizers = new Localizer[5];
  private Encoder rightEncoder, leftEncoder, centerEncoder;
  private Pose2d rightPose, leftPose, centerPose;

  public TestLocalizer(HardwareMap hardwareMap, BNO055IMU imu1, BNO055IMU imu2){
    rightPose = new Pose2d(-0.55, 7.3, 0);
    leftPose = new Pose2d(-0.55, -7.3, 0);
    centerPose = new Pose2d(5.6, 0.512, Math.toRadians(90));

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
    centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFL"));

    leftEncoder.setDirection(Direction.FORWARD);
    rightEncoder.setDirection(Direction.FORWARD);
    centerEncoder.setDirection(Direction.REVERSE);

//    localizers[0] = new RROdometryLocalizerIMU(leftPose, centerPose, 0, centerEncoder, imu1);
//    localizers[1] = new RROdometryLocalizerIMU(leftPose, centerPose, 0, centerEncoder, imu2);
//    localizers[2] = new RROdometryLocalizerIMU(rightPose, centerPose, 2, centerEncoder, imu1);
//    localizers[3] = new RROdometryLocalizerIMU(rightPose, centerPose, 2, centerEncoder, imu2);
//    localizers[4] = new RROdometryLocalizer(hardwareMap);
  }

  @NotNull
  @Override
  public Pose2d getPoseEstimate() {
    return localizers[4].getPoseEstimate();
  }

  @Nullable
  @Override
  public Pose2d getPoseVelocity() {
    return localizers[4].getPoseVelocity();
  }

  @Override
  public void update() {
    localizers[4].update();
  }

  @Override
  public void setPoseEstimate(@NotNull Pose2d pose2d) {

  }

  public Pose2d[] getAllPoseEstimates(){
    return new Pose2d[]{localizers[4].getPoseEstimate()};
  }
}
