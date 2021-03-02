package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.WobbleClaw;

@TeleOp(name = "Wobble Feedforward Diagnostic", group = "Diagnostic")
public class WobbleArmFeedforwardDiagnostic extends OpMode {
  private WobbleClaw claw;
  private GamepadEx pad;
  private PrintWriter writer;
  private Double lastWrite = null;
  private VoltageSensor voltageSensor;

  @Override
  public void init() {
    claw = new WobbleClaw(this);
    claw.armRotator.setMode(RunMode.STOP_AND_RESET_ENCODER);
    pad = new GamepadEx(gamepad1);
    voltageSensor = hardwareMap.voltageSensor.iterator().next();

    try {
      writer = new PrintWriter("/sdcard/FIRST/wobble-diag.csv");
      writer.println("target,current,power,voltage,position");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
      stop();
    }
  }

  @Override
  public void start() {
    claw.armRotator.setTargetPosition(0);
    claw.armRotator.setMode(RunMode.RUN_TO_POSITION);
    claw.armRotator.setPower(0);
    claw.armRotator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
  }

  @Override
  public void loop() {
    if (pad.wasJustPressed(Button.DPAD_DOWN)) {
      claw.armRotator.setTargetPosition(claw.armRotator.getTargetPosition() + 100);
    } else if (pad.wasJustPressed(Button.DPAD_UP)) {
      claw.armRotator.setTargetPosition(claw.armRotator.getTargetPosition() - 100);
    } else if (pad.wasJustPressed(Button.A)) {
      claw.armRotator.setPower(0.2);
    } else if (pad.wasJustPressed(Button.X)) {
      claw.armRotator.setPower(0);
    }

    if (pad.wasJustPressed(Button.LEFT_STICK_BUTTON)) {
      writer.printf("%d,%.5f,%.5f,%.5f,%d\n", claw.armRotator.getTargetPosition(),
          claw.armRotator.getCurrent(CurrentUnit.MILLIAMPS),
          claw.armRotator.getPower(),
          voltageSensor.getVoltage(),
          claw.armRotator.getCurrentPosition());
      lastWrite = getRuntime();
    }

    telemetry.addData("target", claw.armRotator.getTargetPosition());
    telemetry.addData("current (mA)", claw.armRotator.getCurrent(CurrentUnit.MILLIAMPS));
    telemetry.addData("power", claw.armRotator.getPower());
    telemetry.addData("last write", lastWrite == null ? "none" : lastWrite.toString());
    pad.readButtons();
  }

  public void stop() {
    writer.flush();
    writer.close();
  }
}
