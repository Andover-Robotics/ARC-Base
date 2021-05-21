package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.ShootMode;
import java.util.function.DoubleSupplier;

public class ShootingAlignment {
  public static final Pose2d shootingPosition = new Pose2d(-1, -36, 0);
  private RRMecanumDrive drive;

//  private PIDController pidHeading, pidPosition;
//  private DoubleSupplier headingSource = () -> Bot.getInstance().roadRunner.getPoseEstimate().getHeading();

  public ShootingAlignment(RRMecanumDrive drive) {
//    pidHeading = new PIDController(0.8754, 0.7448, 0.032);
//    pidHeading.setSetPoint(0.0);
//    pidHeading.setIntegrationBounds(-1, 1);
//    pidPosition = new PIDController(1, 1, 1);
//    pidPosition.setSetPoint(0.0);
//    pidPosition.setIntegrationBounds(-1, 1);
      this.drive = drive;
  }

  public void runToPositionToggle(){
      if (drive.mode != Mode.IDLE) {
        stopTrajectory();
      } else {
        startTrajectory();
      }
  }

  public void startTrajectory(){
    drive.followTrajectoryAsync(
        drive.trajectoryBuilder(drive.getPoseEstimate()).lineToSplineHeading(shootingPosition)
            .build());
  }

  public void stopTrajectory(){
    drive.mode = Mode.IDLE;
    drive.setDriveSignal(new DriveSignal());
  }

//  public void setHeadingCoefficients(double p, double i, double d) {
//    pidHeading.setPID(p, i, d);
//  }
//
//  public void setPositionCoefficients(double p, double i, double d) { pidPosition.setPID(p, i, d); }
//
//  public double runHeading() {
//    double target = pidHeading.getSetPoint(), current = headingSource.getAsDouble();
//    current = Angle.normDelta(current - target) + target;
//    return pidHeading.calculate(current);
//  }
//
//  public Vector2d runPosition() {
//    Vector2d target = pidPosition.getSet
//  }
//
//  public Pose2d run() {
//    return new Pose2d(runPosition(), runHeading());
//  }
//
//  void reset() {
//    pidHeading.reset();
//  }
//
//  public double getSetPoint() {
//    return pidHeading.getSetPoint();
//  }

//  public double targetRpmByDistance(double d, ShootMode mode) {
//    // See 2021-03-17 entry
//    final double g = 386.0886; // in/s^2
//    final double theta = Math.PI / 6;
//    final double inside = -g*d*d / (2 * (mode.height - 8 - d * Math.tan(theta)) * Math.pow(Math.cos(theta), 2));
//    if (inside < 0) return Double.NaN;
//    final double inPerSec = Math.sqrt(inside);
//    return inPerSec * 2.2 / 2 / (2 * Math.PI) * 60;
//  }

//  private double headingByPosition(Vector2d pos, Vector2d target) {
//    return target.minus(pos).angle();
//  }

//  public void updateTargetAngle(Vector2d pos, Vector2d target) {
//    pidHeading.setSetPoint(headingByPosition(pos, target));
//  }
}
