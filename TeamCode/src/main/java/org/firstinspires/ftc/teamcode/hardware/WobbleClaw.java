package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleClaw extends SubsystemBase {
  public static double
      clawClosedPos = 0.5, clawOpenPos = 0.9, armSpeed = 0.35;
  public static int
      armUpPos = 120, armDownPos = 350, armDropPos = 280, rotationMaxTicksPerIteration = 4;//TODO: test values
  private int armPos = 0;//-156 is position @ "top"

  public DcMotorEx armRotator;
  private Servo claw;

  public WobbleClaw(OpMode opMode) {
    armRotator = opMode.hardwareMap.get(DcMotorEx.class, "wobbleArm");
    armRotator.setTargetPosition(0);
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    armRotator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    armRotator.setDirection(Direction.REVERSE);
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
    armRotator.setPower(0.45);
  }

  public void stopArm() {
    armRotator.setTargetPosition(armRotator.getCurrentPosition());
    armRotator.setPower(0);
  }

  public void open() {
    claw.setPosition(clawOpenPos);
  }

  public void close() {
    claw.setPosition(clawClosedPos);
  }


  private FtcDashboard dash;

  public void periodic() {
    dash.getTelemetry().addData("arm target", armRotator.getTargetPosition());
    dash.getTelemetry().addData("arm position", armRotator.getCurrentPosition());
    dash.getTelemetry().addData("arm power", armRotator.getPower());
    dash.getTelemetry().update();
  }

  public void raiseArm() {
    armRotator.setTargetPosition(armUpPos);
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    armRotator.setPower(0.2);
  }

  public void lowerArmToDrop() {
    armRotator.setTargetPosition(armDropPos);
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    armRotator.setPower(0.18);
  }

  public void lowerArm() {
    armRotator.setTargetPosition(armDownPos);
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    armRotator.setPower(0.2);
  }

  public void stowArm() {
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    armRotator.setTargetPosition(0);
    armRotator.setPower(0.2);
  }
  // Autonomous specific
  public void waitUntilTargetReached(LinearOpMode opMode) {
    double startTime = opMode.getRuntime();
    while (Math.abs(armRotator.getCurrentPosition() - armRotator.getTargetPosition()) > 20 ||
        armRotator.isBusy() && !opMode.isStopRequested()) {
      if (opMode.getRuntime() - startTime > 3) break;
    }
  }
}
