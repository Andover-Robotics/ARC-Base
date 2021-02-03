package org.firstinspires.ftc.teamcode.demo;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.WobbleClaw;

@TeleOp(name = "Wobble claw demo", group = "Experimental")
public class WobbleClawDemo extends OpMode {

  private WobbleClaw claw;
  private MecanumDrive drive;
  private boolean lowSpeedMode = true;
  private double driveSpeed = 0.4;

  @Override
  public void init() {
    claw = new WobbleClaw(this);
    drive = new MecanumDrive(true,
        new Motor(hardwareMap, "motorFL"),
        new Motor(hardwareMap, "motorFR"),
        new Motor(hardwareMap, "motorBL"),
        new Motor(hardwareMap, "motorBR"));
  }

  @Override
  public void init_loop() {
    telemetry.addData("clawOpenPos", WobbleClaw.clawOpenPos);
    telemetry.addData("clawClosedPos", WobbleClaw.clawClosedPos);
  }

  @Override
  public void loop() {
//    claw.setArmPower(
//        Math.abs(gamepad1.right_trigger) > Math.abs(gamepad1.left_trigger) ? gamepad1.right_trigger
//            : -gamepad1.left_trigger);
//    claw.setArmPower(-gamepad1.right_stick_y);

    if (gamepad1.right_bumper) {
      claw.close();
    } else {
      claw.open();
    }
    if (lowSpeedMode) {
      drive
          .driveRobotCentric(driveSpeed * gamepad1.left_stick_x,
          -driveSpeed * gamepad1.left_stick_y, driveSpeed * gamepad1.right_stick_x);
    } else {
      drive
          .driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    if (gamepad1.a) {
      lowSpeedMode = true;
    } else if (gamepad1.b) {
      lowSpeedMode = false;
    }
    telemetry.addData("low speed", lowSpeedMode);
  }
}