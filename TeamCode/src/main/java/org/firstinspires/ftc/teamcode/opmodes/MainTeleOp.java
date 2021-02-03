package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {

  enum ShootMode {
    POWER_SHOT,
    TOWER_GOAL
  }

  enum TowerMode {
    HIGH,
    MIDDLE,
    LOW
  }

  private ShootMode shootState = ShootMode.TOWER_GOAL;
  private TowerMode towerState = TowerMode.HIGH;

  /*
    Guidance:
    - Should use module classes from the module package
    - Should make a "Bot" class
    - Should use MecanumDrive from FTCLib; we don't implement mecanum ourselves
   */
  void subInit(){

  }

  void buttonsInit() {
    toggleButtonReaders.put("g2a", new ToggleButtonReader(gamepadEx2, Button.A));
    toggleButtonReaders.put("g2b", new ToggleButtonReader(gamepadEx2, Button.B));
  }

  @Override
  public void subLoop() {
//  Controller 1	(Movement)
//
//    Left joystick		Move (Strafing)
//    Right joystick		Independent rotation (Locks if Strafe enabled)
    //    L. Bumper		Rotation Lock
//    R. Bumper		Rotation Lock
//    L. Trigger		Adjust Slow Mode
//    R. Trigger		Adjust Slow Mode
    driveSpeed = (1 - 0.4 * (gamepad1.left_trigger + gamepad1.right_trigger));
    bot.drive.driveFieldCentric(gamepad1.left_stick_x * driveSpeed,
        -gamepad1.left_stick_y * driveSpeed,
        gamepad1.left_bumper || gamepad1.right_bumper ? 0 : gamepad2.right_stick_x,
        bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
            AngleUnit.DEGREES).firstAngle);//may have to switch angle

//    D-pad			Move (Strafing at 100%) optional

//    A 			N/A
//    B 			N/A
//    X 			Line up with tower goal automatically
//    Y 			Feed single ring to shooter
//    if(gamepadEx2.wasJustPressed(Button.X)){
    // TODO line up with tower goal
//    }

    if (gamepadEx1.wasJustPressed(Button.Y)) {
      bot.shooter.feedRing();
    }

//    Start			N/A
//    Select			N/A
//    Logitech button	Party mode
//

//
//Controller 2	(Robot tools)
//
//    Left joystick		Rotate useless servo with decoration on it
//    Right joystick		N/A
//
//    D-pad			Up: Raise wobble arm at constant speed; Down: Lower
//    wobble arm at constant speed;

    if (gamepad2.dpad_down) {
      bot.wobbleClaw.rotateArm(-0.1);
    } else if (gamepad2.dpad_up) {
      bot.wobbleClaw.rotateArm(0.1);
    } else {
      bot.wobbleClaw.stopArm();
    }

    //    A			Toggle intake; toggle shooter between idle and shooting
//    B			Toggle Wobble Goal Claw
//    X			N/A
//    Y			Feed single ring to shooter

    if (gamepadEx2.getButton(Button.X)) {
      bot.intake.stop();
      bot.shooter.turnOff();
    } else if (toggleButtonReaders.get("g2a").wasJustReleased()) {
      if (toggleButtonReaders.get("g2a").getState()) {
        bot.intake.run();
        bot.shooter.runIdleSpeed();
      } else {
        bot.intake.stop();
        bot.shooter.runShootingSpeed();
      }
    }


    if (toggleButtonReaders.get("g2b").wasJustReleased()) {
      if (toggleButtonReaders.get("g2b").getState()) {
        bot.wobbleClaw.open();
      } else {
        bot.wobbleClaw.close();
      }
    }

    if (gamepadEx2.wasJustPressed(Button.Y)) {
      bot.shooter.feedRing();
    }
//
//    Start			N/A
//    Select			N/A
//
//    L. Bumper		Toggle Power Shot mode
//    R. Bumper		Cycle between High, Mid, and Low Goal
//    L. Trigger		N/A
//    R. Trigger		N/A
    if (gamepadEx2.wasJustPressed(Button.LEFT_BUMPER)) {
      if (shootState == ShootMode.POWER_SHOT) {
        shootState = ShootMode.TOWER_GOAL;
      } else {
        shootState = ShootMode.POWER_SHOT;
      }
    }

    if (gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER)) {
      cycleTowerState();
    }

    CommandScheduler.getInstance().run();
  }

  private void cycleTowerState() {
    switch (towerState) {
      case LOW:
        towerState = TowerMode.HIGH;
        break;
      case HIGH:
        towerState = TowerMode.MIDDLE;
        break;
      case MIDDLE:
        towerState = TowerMode.LOW;
        break;
    }
  }

  void updateTelemetry() {
    telemetry.addData("Left Joystick (Strafing) X-Val: ", gamepad1.left_stick_x);
    telemetry.addData("Right Joystick (Turning) X-Val: ", gamepad1.right_stick_x);
    telemetry.addData("Left Joystick (Moving) Y-Val: ", gamepad1.left_stick_y);
  }
}
