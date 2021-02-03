package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import java.util.HashMap;
import java.util.Map;

/**
 * Make a TeleOp OpMode for a robot with the following hardware devices:
 *  - Mecanum drive base (motorFL, motorFR, motorBL, motorBR)
 *  - Intake roller (intake)
 *  - Shooter flywheel (shooter, full speed launches rings)
 *
 *  Your OpMode should enable the driver to drive the robot holonomically (i.e. moving in any direction),
 *  operate the intake, and operate the shooter. Bonus if you can automate some of this.
 *
 *  You design the control scheme.
 */
public abstract class BaseOpMode extends OpMode {

  Bot bot;
  GamepadEx gamepadEx1, gamepadEx2;

  //button reader syntax
  // (g1 or g2)  (a, b, lt, lb, etc)
  Map<String, ToggleButtonReader> toggleButtonReaders = new HashMap<>();
  double driveSpeed;

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    gamepadEx2 = new GamepadEx(gamepad2);
    gamepadEx1 = new GamepadEx(gamepad1);
    buttonsInit();
    subInit();
    telemetry.addLine("Init done");
    telemetry.update();
  }

  @Override
  public void loop() {
    updateButtons();
    updateTelemetry();
    subLoop();
  }

  abstract void subInit();
  abstract void buttonsInit();

  abstract void subLoop();
  abstract void updateTelemetry();

  void updateButtons(){
    for(ToggleButtonReader t : toggleButtonReaders.values()){
      t.readValue();
    }
    gamepadEx1.readButtons();
    gamepadEx2.readButtons();
  }
}
