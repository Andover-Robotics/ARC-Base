package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;

public class ShootingAlignment {
  public static double shootingAngle = -90.0;

  private PIDController pid;

  public ShootingAlignment() {
    pid = new PIDController(0.01528, 0.0125, 5.5555e-4);
    pid.setSetPoint(shootingAngle);
  }

  public void setCoefficients(double p, double i, double d) {
    pid.setPID(p, i, d);
  }

  public double run(double input) {
    return pid.calculate(input > 92 ? input-360 : input);
  }

  void reset() {
    pid.reset();
  }
}
