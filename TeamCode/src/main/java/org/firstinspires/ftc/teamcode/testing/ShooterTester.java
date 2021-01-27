package org.firstinspires.ftc.teamcode.testing;

import android.os.Build.VERSION_CODES;
import android.os.Environment;
import android.util.Log;
import androidx.annotation.RequiresApi;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.stream.Collectors;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * User Will Be Able To:
 * Shoot rings from shooter
 * Change parameters as they please
 *
 * <ul>
 *   <li>Adjust the shooter motor's RPM in small increments from 0.7 to 1.0</li>
 *   <li>See a graph of the shooter motor's RPM</li>
 *   <li>See automated measurements of the drop in RPM immediately after shooting and the time
 *    taken to spool back up to shooting speed</li>
 * </ul>
 */
@TeleOp(name = "ShooterTester", group = "Experimental")
public class ShooterTester extends OpMode {
  private MotorEx shooter;
  private MecanumDrive drive;
  private ToggleButtonReader aToggle;
  private ButtonReader speedUp;
  private ButtonReader speedDown;
  private Telemetry dashTelemetry;
  private StreamingDataWriter dataWriter;
  private ElapsedTime timer = new ElapsedTime(Resolution.MILLISECONDS);

  // If null, not dropping; if not null, then dropping
  private Double dropInitialSpeed = null;
  private Double dropInitialTime = null;
  private double lastAccel = 1000;
  private double lastSpeed = 0;
  private double lastMs;

  private double shootingSpeed = 0.7;

  private static class TrialRpmResults {
    final double recoveryTime, speedDrop;

    public TrialRpmResults(double recoveryTime, double speedDrop) {
      this.recoveryTime = recoveryTime;
      this.speedDrop = speedDrop;
    }
  }

  public void init() {
    dashTelemetry = FtcDashboard.getInstance().getTelemetry();

    final GamepadEx gamepad = new GamepadEx(gamepad1);
    aToggle = new ToggleButtonReader(gamepad, Button.X);
    speedUp = new ButtonReader(gamepad, Button.DPAD_UP);
    speedDown = new ButtonReader(gamepad, Button.DPAD_DOWN);
    shooter = new MotorEx(hardwareMap, "shooter");
    shooter.setInverted(true);

    drive = new MecanumDrive(true,
        new Motor(hardwareMap, "motorFL"),
        new Motor(hardwareMap, "motorFR"),
        new Motor(hardwareMap, "motorBL"),
        new Motor(hardwareMap, "motorBR"));
    drive.setMaxSpeed(0.5);

    try {
      dataWriter = new StreamingDataWriter(20,
          () -> new double[] { shooter.getVelocity(), shootingSpeed },
          "/sdcard/FIRST/shooterdata.csv");
    } catch (Exception e) {
      System.out.println("File not found, please stop being dumb");
      stop();
    }
  }

  List<TrialRpmResults> dataSet = new ArrayList<>();

  public void start() {
    dataWriter.startStreaming();
    timer.reset();
    lastMs = timer.time();
  }

  public void loop() {
    aToggle.readValue();
    if (aToggle.getState()) {
      shooter.set(shootingSpeed);
    } else {
      shooter.set(0);
    }

    // v => Ticks per second
    // v / 145.6 => revolutions of motor shaft per second
    // v / 145.6 * 6 => revolutions of flywheel per second
    // v / 145.6 * 6 * 2pi => radians of flywheel rotation per second
    // v / 145.6 * 6 * 2pi * 2 => tangential velocity of flywheel (in inches per second)
    double motorSpeed = shooter.getVelocity() / GoBILDA.RPM_1150.getCPR() * 6 * 2 * Math.PI * 2;

    // Approximate tangential acceleration and jerk to isolate drop
    double dt = timer.time() - lastMs;
    lastMs += dt;
    double dv = motorSpeed - lastSpeed;
    lastSpeed += dv;
    double motorAccel = dv / dt;
    double motorJerk = (motorAccel - lastAccel) / dt;
    lastAccel = motorAccel;

    if (dropInitialSpeed != null && motorJerk > 20) {
      Log.i("ShooterTester", "RPM drop complete");
      dataSet.add(new TrialRpmResults(
          timer.time() - dropInitialTime,
          motorSpeed - dropInitialSpeed));
      dropInitialSpeed = dropInitialTime = null;

    } else if (dropInitialSpeed == null && motorJerk < 20) {
      Log.i("ShooterTester", "RPM drop started");
      dropInitialSpeed = motorSpeed;
      dropInitialTime = timer.time();
    }

    speedDown.readValue();
    speedUp.readValue();
    if (speedUp.wasJustPressed() && shootingSpeed < 1) {
      shootingSpeed += 0.02;
    }
    if (speedDown.wasJustPressed() && shootingSpeed > 0.7) {
      shootingSpeed -= 0.02;
    }

    drive.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

    telemetry.addData("target shooting speed", shootingSpeed);
    dashTelemetry.addData("current shooting speed", motorSpeed);
    dashTelemetry.update();
    telemetry.addData("drops detected", dataSet.size());
    telemetry.addData("currently dropping?", dropInitialSpeed != null);
    telemetry.addData("cycle rate (Hz)", 1000 / dt);

    telemetry.addLine("controls")
        .addData("toggle shooter", "X");
  }

  public void stop() {
    dataWriter.stopStreaming();

    try {
      PrintWriter writer = new PrintWriter("/sdcard/FIRST/rpmDrops.csv");
      for (TrialRpmResults result : dataSet) {
        writer.printf("%.5f,%.5f\n", result.recoveryTime, result.speedDrop);
      }
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}