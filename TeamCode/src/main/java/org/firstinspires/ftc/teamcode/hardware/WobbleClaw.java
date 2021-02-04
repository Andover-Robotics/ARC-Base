package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleClaw extends SubsystemBase {
  public static double
      clawClosedPos = 0.25, clawOpenPos = 0.6, armSpeed = 0.2;
  public static int
      armUpPos = 230, armDownPos = 406, rotationMaxTicksPerIteration = 4;//TODO: test values
  private int armPos = 0;//-156 is position @ "top"

  private MotorEx armRotator;
  private Servo claw;

  public WobbleClaw(OpMode opMode) {
    armRotator = new MotorEx(opMode.hardwareMap, "wobbleArm", GoBILDA.RPM_435);
    armRotator.setTargetPosition(0);
    armRotator.setRunMode(RunMode.PositionControl);
    armRotator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    armPos = armRotator.getCurrentPosition();

    claw = opMode.hardwareMap.servo.get("claw");
    dash = FtcDashboard.getInstance();
  }

  public void rotateArm(double velocity) {
    armPos = armRotator.getCurrentPosition();
//    armPos += velocity * rotationMaxTicksPerIteration;
//    armPos = Math.min(armPos, armUpPos);
//    armPos = Math.max(armPos, armDownPos);
    armRotator.setTargetPosition(
        (int)Math.round(armRotator.getCurrentPosition() + velocity * rotationMaxTicksPerIteration));
  }

  public void stopArm() {
//    armRotator.setTargetPosition(armPos);
  }

  public void open() {
    claw.setPosition(clawOpenPos);
  }

  public void close() {
    claw.setPosition(clawClosedPos);
  }


  private FtcDashboard dash;
  @Override
  public void periodic() {
    armRotator.set(armSpeed);
    dash.getTelemetry().addData("arm target", armRotator.atTargetPosition());
    dash.getTelemetry().addData("arm position", armRotator.getCurrentPosition());
    dash.getTelemetry().addData("arm power", armRotator.get());
  }

  public void raiseArm() {
    armRotator.setTargetPosition(armUpPos);
  }

  public void lowerArm() {
    armRotator.setTargetPosition(armDownPos);
  }
  // Autonomous specific
//  public void rotateUntilTargetReached(LinearOpMode opMode) {
//    while (armRotator.isBusy() && !opMode.isStopRequested()) {
//      armRotator.setPower(armSpeed);
//    }
//  }
//
}
