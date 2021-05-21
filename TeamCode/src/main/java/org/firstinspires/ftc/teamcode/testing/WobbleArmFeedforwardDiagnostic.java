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
  private StreamingDataWriter writer;
  private VoltageSensor voltageSensor;

  @Override
  public void init() {
    claw = new WobbleClaw(this);
    claw.armRotator.setMode(RunMode.STOP_AND_RESET_ENCODER);
    pad = new GamepadEx(gamepad1);
    voltageSensor = hardwareMap.voltageSensor.iterator().next();

    try {
      writer = new StreamingDataWriter(20, () -> new double[] {
          getRuntime(),
          claw.armRotator.getPower(),
          claw.armRotator.getCurrentPosition(),
          claw.armRotator.getCurrent(CurrentUnit.AMPS),
          claw.armRotator.getVelocity(),
          voltageSensor.getVoltage()
      }, "/sdcard/FIRST/wobble-diag.csv");
      writer.writeLine("time,power,pos,current,vel,voltage");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
      stop();
    }
  }

  @Override
  public void start() {
    writer.startStreaming();
    claw.armRotator.setTargetPosition(0);
    claw.armRotator.setMode(RunMode.RUN_WITHOUT_ENCODER);
    claw.armRotator.setPower(0);
    claw.armRotator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
  }

  boolean clawClosed = false;
  @Override
  public void loop() {
    claw.armRotator.setPower(gamepad1.left_stick_y * 0.4 + gamepad1.right_stick_y * 0.4);
    if (pad.wasJustPressed(Button.X)) {
      clawClosed = !clawClosed;
      if (clawClosed) claw.close();
      else claw.open();
    }

    pad.readButtons();
  }

}
