package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleClaw extends SubsystemBase {
  public static double
      clawClosedPos = 0.25, clawOpenPos = 0.6, armSpeed = 0.2;
  public static int
      armUpPos = 230, armDownPos = 406, rotationMaxTicksPerIteration = 4;//TODO: test values
  private int armPos = 0;//-156 is position @ "top"

  private DcMotor armRotator;
  private Servo claw;

  public WobbleClaw(OpMode opMode) {
    armRotator = opMode.hardwareMap.dcMotor.get("wobbleArm");
    armRotator.setMode(RunMode.RUN_USING_ENCODER);
    armRotator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    armPos = armRotator.getCurrentPosition();

    claw = opMode.hardwareMap.servo.get("claw");
    dash = FtcDashboard.getInstance();
  }

  public void rotateArm(double velocity) {
//    armPos = armRotator.getCurrentPosition();
////    armPos += velocity * rotationMaxTicksPerIteration;
////    armPos = Math.min(armPos, armUpPos);
////    armPos = Math.max(armPos, armDownPos);
//    armRotator.setTargetPosition(
//        (int)Math.round(armRotator.getCurrentPosition() + velocity * rotationMaxTicksPerIteration));
    armRotator.setPower(velocity * 0.25);
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

  public void periodic() {
//    armRotator.setPower(armSpeed);
    dash.getTelemetry().addData("arm target", armRotator.getTargetPosition());
    dash.getTelemetry().addData("arm position", armRotator.getCurrentPosition());
    dash.getTelemetry().addData("arm power", armRotator.getPower());
  }

  public void raiseArm() {
    armRotator.setTargetPosition(armUpPos);
  }

  public void lowerArm() {
    armRotator.setTargetPosition(armDownPos);
  }
  // Autonomous specific
  public void waitUntilTargetReached(LinearOpMode opMode) {
    while (armRotator.isBusy() && !opMode.isStopRequested()) {
      armRotator.setPower(armSpeed);
    }
  }
}
