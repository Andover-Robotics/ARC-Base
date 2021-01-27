package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
  public static double onPower = 0.7, offPower = 0.1;
  public static double magazineBackward = 0, magazineForward = 70;//TODO: test values
  public static long timerLength = 690;
  private MotorEx motor;
  private ServoEx magazine;
  private Timer feederTimer = null;

  public Shooter(OpMode opMode){
    motor = new MotorEx(opMode.hardwareMap, "shooter", GoBILDA.RPM_1150);
    motor.setRunMode(RunMode.VelocityControl);
    motor.set(0);
    motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

    magazine = new SimpleServo(opMode.hardwareMap, "magazine", 0, 300);
    magazine.turnToAngle(magazineBackward);
  }

  public void feedRing() {
    magazine.turnToAngle(magazineForward);
    feederTimer = new Timer(timerLength, TimeUnit.MILLISECONDS);
    feederTimer.start();
  }

  @Override
  public void periodic() {
    if (feederTimer != null) //Jank code
      if (feederTimer.done()) {
        magazine.turnToAngle(magazineBackward);
        feederTimer = null;
      }
  }

  public void runShootingSpeed(){
    motor.set(onPower);
  }

  public void runIdleSpeed(){
    motor.set(offPower);
  }
}
