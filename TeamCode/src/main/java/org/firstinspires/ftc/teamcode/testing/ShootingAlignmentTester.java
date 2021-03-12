package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmodes.ShootingAlignment;
import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "Shooting alignment tester", group = "Experimental")
public class ShootingAlignmentTester extends OpMode {
  double p = 0.01528, i = 0.0125, d = 6e-4;
  private static final double step = 1 / 180.0 / 20.0;
  private Bot bot;
  private GamepadEx gamepadEx;
  private ShootingAlignment alignment = new ShootingAlignment();
  private double lastReadButtons = 0;
  private FtcDashboard dash;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    gamepadEx = new GamepadEx(gamepad1);
    dash = FtcDashboard.getInstance();
  }

  @Override
  public void init_loop() {
    super.init_loop();
  }

  @Override
  public void start() {
    super.start();
    alignment.setCoefficients(p, i, d);
  }

  @Override
  public void loop() {
    readGamepad();
    showCoefficients();
    double gyroAngle = bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
    double turnAngle = gamepad1.dpad_right ? -alignment.run(gyroAngle) : gamepad1.right_stick_x;
    bot.drive.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), turnAngle, gyroAngle);
    dash.getTelemetry().addData("error", ShootingAlignment.shootingAngle - gyroAngle);
    dash.getTelemetry().update();
    telemetry.addData("error", ShootingAlignment.shootingAngle - gyroAngle);
    telemetry.update();
  }
  
  private void readGamepad() {
    if (getRuntime() - lastReadButtons > 0.2) {
      if (gamepad1.dpad_up) {
        p += step;
      } else if (gamepad1.dpad_down) {
        p -= step;
      } else if (gamepad1.y) {
        i += step;
      } else if (gamepad1.a) {
        i -= step;
      } else if (gamepad1.left_stick_y < -0.5) {
        d += step;
      } else if (gamepad1.left_stick_y > 0.5) {
        d -= step;
      }
      lastReadButtons = getRuntime();
      alignment.setCoefficients(p, i, d);
    }
  }


  private void showCoefficients() {
    telemetry.addData("p", p)
        .addData("i", i)
        .addData("d", d);
  }

}
