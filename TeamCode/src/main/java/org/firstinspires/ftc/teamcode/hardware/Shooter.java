package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
  public static double onPower = 0.9, offPower = 0.1;
  public static double magazineBackward = 0.705, magazineForward = 0.88;

  private enum State {
    OFF(0), IDLE(offPower), SHOOT(onPower);
    public final double power;
    State(double power) {
      this.power = power;
    }
  }

  // 5400 rev/min * 1/60 min/second * 28 ticks/rev => ticks/second
  private static int MAX_TICKS_PER_SECOND = 5400 / 60 * 28;

  public static long timerLength = 500;
  private MotorEx motor;
  private ServoEx magazine;
  private Timer feederTimer = null;
  private final int targetFlywheelTicksPerSecond;
  private State state = State.OFF;

  private static FtcDashboard dash;

  public Shooter(OpMode opMode){
    motor = new MotorEx(opMode.hardwareMap, "shooter", GoBILDA.RPM_1150);
//    motor.setRunMode(RunMode.VelocityControl);
    // using built-in velocity controller for now because FTCLib 1.2.0-beta is broken
    motor.motorEx.setMode(RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    // TODO adjust this according to real world tests as the flywheel disqualifies "no-load max RPM"
    targetFlywheelTicksPerSecond = (int)Math.round(MAX_TICKS_PER_SECOND * onPower);

    magazine = new SimpleServo(opMode.hardwareMap, "magazine", 0, 300);
    magazine.setPosition(magazineBackward);

    dash = FtcDashboard.getInstance();
  }

  public void feedRing() {
    magazine.setPosition(magazineForward);
    feederTimer = new Timer(timerLength, TimeUnit.MILLISECONDS);
    feederTimer.start();
  }

  @Override
  public void periodic() {
    if (feederTimer != null)
      if (feederTimer.done()) {
        magazine.setPosition(magazineBackward);
        feederTimer = null;
      }
    dash.getTelemetry().addData("shooter velocity", motor.getVelocity());
      dash.getTelemetry().update();
    motor.motorEx.setPower(state.power);
  }

  public void runShootingSpeed(){
    state = State.SHOOT;
  }

  public void runIdleSpeed(){
    state = State.IDLE;
  }

  public void turnOff() {
    state = State.OFF;
  }

  // Autonomous functions

  public void shootRings(LinearOpMode opMode, int numRings, double vel) {
    for (int i = 0; i < numRings; i++) {
      shootOneRing(opMode, vel);
      // Wait for the next ring to fall
      opMode.sleep(400);
    }
  }

  public void shootOneRing(LinearOpMode opMode, double vel) {
    // intent: pivot the feeder, wait for a moment, wait for the shooter rpm to rebound, pivot the feeder back
    magazine.setPosition(magazineForward);
    opMode.sleep(700);
    magazine.setPosition(magazineBackward);
    double shootTime = opMode.getRuntime();

    while (true) {
      if (opMode.isStopRequested()) return;
      double timePassed = opMode.getRuntime() - shootTime;
      if (timePassed > 2) break;
      if (isFlywheelAtTargetVelocity(vel) && timePassed > 0.5) break;
    }
  }

  public boolean isFlywheelAtTargetVelocity(double target) {
    // tolerance: 5 rev/min * 1/60 min/sec * 28 ticks/rev
    return Math.abs(MAX_TICKS_PER_SECOND * target - motor.getVelocity()) < 5.0 / 60 * 28;
  }
}
