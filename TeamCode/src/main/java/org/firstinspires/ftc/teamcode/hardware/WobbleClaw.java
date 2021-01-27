package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleClaw extends SubsystemBase {
  public static double
      clawClosedPos = 0.25, clawOpenPos = 0.6, armSpeed = 0.15;
  public static int
      armUpPos = -300, armDownPos = 0, rotationMaxTicksPerIteration = 3;//TODO: test values
  private int armPos = 0;

  private MotorEx armRotator;
  private Servo claw;

  public WobbleClaw(OpMode opMode) {
    armRotator = new MotorEx(opMode.hardwareMap, "wobbleArm", GoBILDA.RPM_435);
    armRotator.setRunMode(Motor.RunMode.PositionControl);
    armRotator.setTargetPosition(0);

    claw = opMode.hardwareMap.servo.get("claw");
  }

  public void rotateArm(double velocity) {
    armPos += velocity * rotationMaxTicksPerIteration;
    armRotator.setTargetPosition(armPos);
  }


  //4 bar code
  public void open() {
    claw.setPosition(clawOpenPos);
  }

  public void close() {
    claw.setPosition(clawClosedPos);
  }

  @Override
  public void periodic() {
    armRotator.set(armSpeed);
  }

  public void raiseArm() {
    armRotator.setTargetPosition(armUpPos);
  }

  public void lowerArm() {
    armRotator.setTargetPosition(armDownPos);
  }

  private int clamp(int value, int min, int max) {
    return value < min ? min : Math.min(value, max);
  }
}
