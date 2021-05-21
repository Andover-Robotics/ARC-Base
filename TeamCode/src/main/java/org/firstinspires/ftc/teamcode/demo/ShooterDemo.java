package org.firstinspires.ftc.teamcode.demo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@TeleOp(name = "Shooter demo", group = "Experimental")
public class ShooterDemo extends OpMode {
  boolean on = false;
  MotorEx motor;
  double p = 15.0, i = 1.5, d = 0.2, f = 0;
  double lastUpdateTime = 0;

  @Override
  public void init() {
    motor = new MotorEx(hardwareMap, "shooter", GoBILDA.RPM_1150);
//    motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    motor.motorEx.setMode(RunMode.RUN_USING_ENCODER);
  }

  @Override
  public void init_loop() {
  }

  @Override
  public void loop() {
    if(gamepad1.b){
      motor.motorEx.setVelocity(28.0 * 3200.0 / 60.0);
    }
    if(gamepad1.x){
      motor.motorEx.setVelocity(0);
    }
    if (getRuntime() - lastUpdateTime > 0.3) {
      if (gamepad1.dpad_up) {
        p += 0.5;
      } else if (gamepad1.dpad_down) {
        p -= 0.5;
      }
      if (gamepad1.y) {
        i += 0.5;
      } else if (gamepad1.a) {
        i -= 0.5;
      }
      if (gamepad1.left_stick_y < -0.5) {
        d += 0.5;
      } else if (gamepad1.left_stick_y > 0.5) {
        d -= 0.5;
      }
      motor.motorEx.setVelocityPIDFCoefficients(p, i, d, f);
      lastUpdateTime = getRuntime();
    }
    telemetry.addData("p", p)
        .addData("i", i)
        .addData("d", d)
        .addData("f", f);

    FtcDashboard.getInstance().getTelemetry().addData("velocity", motor.motorEx.getVelocity());
    FtcDashboard.getInstance().getTelemetry().update();
  }
}
