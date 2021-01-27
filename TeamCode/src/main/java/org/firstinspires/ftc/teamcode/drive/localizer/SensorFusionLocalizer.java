package org.firstinspires.ftc.teamcode.drive.localizer;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;


/*
Encoder counts from three perpendicular dead-wheels (odometry)
Angular orientation, angular acceleration, and linear acceleration from two Inertial Measurement Units (IMU)
Measurements from the Robot Controller phone's gyro sensor
 */
public abstract class SensorFusionLocalizer implements Localizer, SensorEventListener {
  protected Localizer deadWheelLocalizer;
  protected BNO055IMU revGyro1, revGyro2;//getAngularOrientat
  protected SensorManager sensorManager;
  protected Sensor accelerometer;  //accelerometer, light, proximity, magnet, compass, -gyro-, baro

  public SensorFusionLocalizer(Context context, Localizer deadWheelLocalizer, BNO055IMU revGyro1, BNO055IMU revGyro2) {
    sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);//bruh
    this.deadWheelLocalizer = deadWheelLocalizer;
    this.revGyro1 = revGyro1;
    this.revGyro2 = revGyro2;
    this.accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
  }
}
