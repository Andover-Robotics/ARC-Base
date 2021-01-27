package org.firstinspires.ftc.teamcode.drive.localizer;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class WAvgOffsetLocalizer extends SensorFusionLocalizer{
  //I just realized this is kinda proof by induction
  //We want to prove sensor fusion estimate = real position

  //Assume sensor fusion estimate = real position
  //sensor fusion estimate = f(Localizer)
  //Localizer = g(sensor fusion estimate, Localizer)
  //Therefore, sensor fusion estimate = f(g(sensor fusion estimate, Localizer))
  //Since sensor fusion estimate = real position,
  //sensor fusion estimate = f(g(real position, Localizer))
  //and sensor fusion estimate = real position
  //⬛ QED
  public WAvgOffsetLocalizer(Context context, Localizer deadWheelLocalizer, BNO055IMU revGyro1,
      BNO055IMU revGyro2) {
    super(context, deadWheelLocalizer, revGyro1, revGyro2);
  }

  @NotNull
  @Override
  public Pose2d getPoseEstimate() {
    return null;
  }

  @Override
  public void setPoseEstimate(@NotNull Pose2d pose2d) {

  }

  @Nullable
  @Override
  public Pose2d getPoseVelocity() {
    return null;
  }

  @Override
  public void update() {

  }

  @Override
  public void onSensorChanged(SensorEvent sensorEvent) {

  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int i) {

  }
}
