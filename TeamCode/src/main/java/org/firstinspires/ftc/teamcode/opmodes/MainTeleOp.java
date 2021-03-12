package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Pair;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.function.Function;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {
  double cycle = 0;
  double prevRead = 0;
  double fieldCentricOffset = 90.0;

  enum ShootMode {
    POWER_SHOT,
    TOWER_GOAL
  }

  enum TowerMode {
    HIGH,
    MIDDLE,
    LOW
  }

  enum RingMode {
    INTAKE,
    SHOOT,
    OFF
  }

  private ShootMode shootState = ShootMode.TOWER_GOAL;
  private TowerMode towerState = TowerMode.HIGH;
  private RingMode ringMode = RingMode.INTAKE;
  private ShootingAlignment aligner = new ShootingAlignment();
  private boolean clawOpen = true;

  void subInit() {

  }

  void buttonsInit() {
    toggleButtonReaders.put("g2a", new ToggleButtonReader(gamepadEx2, Button.A));
    toggleButtonReaders.put("g2b", new ToggleButtonReader(gamepadEx2, Button.B));
  }

  @Override
  public void subLoop() {
    cycle = 1.0/(time-prevRead);
    prevRead = time;

//  Controller 1	(Movement)
//
//    Left joystick		Move (Strafing)
//    Right joystick		Independent rotation (Locks if Strafe enabled)
    //    L. Bumper		Rotation Lock
//    R. Bumper		Rotation Lock
//    L. Trigger		Adjust Slow Mode
//    R. Trigger		Adjust Slow Mode
    driveSpeed = (1 - 0.4 * (triggerSignal(Trigger.LEFT_TRIGGER) + triggerSignal(Trigger.RIGHT_TRIGGER)));
    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle - fieldCentricOffset;
    Vector2d driveVector = stickSignal(Direction.LEFT),
        turnVector = new Vector2d(buttonSignal(Button.DPAD_RIGHT) ?
            -aligner.run(gyroAngle) :
            stickSignal(Direction.RIGHT).getX() * Math.abs(stickSignal(Direction.RIGHT).getX()), 0);

    bot.drive.driveFieldCentric(
        driveVector.getX() * driveSpeed,
        driveVector.getY() * driveSpeed,
        buttonSignal(Button.LEFT_BUMPER) || buttonSignal(Button.RIGHT_BUMPER) ?
            0 : turnVector.getX() * driveSpeed,
        gyroAngle);
    if (buttonSignal(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset = bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }


//
//Controller 2	(Robot tools)
//
//    Left joystick		Rotate useless servo with decoration on it
//    Right joystick		N/A
//
//    D-pad			Up: Raise wobble arm at constant speed; Down: Lower
//    wobble arm at constant speed;

    if (buttonSignal(Button.DPAD_UP)) {
      bot.wobbleClaw.raiseArm();
    } else if (buttonSignal(Button.DPAD_DOWN)) {
      bot.wobbleClaw.lowerArm();
    }

    //    A			Toggle intake; toggle shooter between idle and shooting
//    B			Toggle Wobble Goal Claw
//    X			stop
//    Y			Feed single ring to shooter

    if (buttonSignal(Button.X) && ringMode != RingMode.OFF) {
      ringMode = RingMode.OFF;
      bot.intake.stop();
      bot.shooter.turnOff();
      bot.wobbleClaw.stopArm();

    } else if (justPressed(Button.A)) {
      ringMode = ringMode == RingMode.INTAKE ? RingMode.SHOOT : RingMode.INTAKE;
    }

    if (buttonSignal(Button.RIGHT_STICK_BUTTON)) {
      bot.intake.spit();
    } else if (ringMode == RingMode.INTAKE) {
      bot.intake.run();
      bot.shooter.runIdleSpeed();
    } else if (ringMode == RingMode.SHOOT) {
      bot.intake.stop();
      bot.intake.convBelt.set(0.6);
      bot.shooter.runShootingSpeed();
    }

//    if (toggleButtonReaders.get("g2b").wasJustReleased()) {
//      if (toggleButtonReaders.get("g2b").getState()) {
//        bot.wobbleClaw.open();
//      } else {
//        bot.wobbleClaw.close();
//      }
//    }
    if (justPressed(Button.DPAD_LEFT)) {
      clawOpen = !clawOpen;
      if (clawOpen) {
        bot.wobbleClaw.open();
      } else {
        bot.wobbleClaw.close();
      }
    }

    if (buttonSignal(Button.Y)) {
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


//    if (gamepadEx2.wasJustPressed(Button.LEFT_BUMPER)) {
//      if (shootState == ShootMode.POWER_SHOT) {
//        shootState = ShootMode.TOWER_GOAL;
//      } else {
//        shootState = ShootMode.POWER_SHOT;
//      }
//    }
//
//    if (gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER)) {
//      cycleTowerState();
//    }

    CommandScheduler.getInstance().run();
  }

  // ------------------------------

  private void reportLocalization() {
    Pose2d poseEstimate = bot.roadRunner.getPoseEstimate();
    telemetry.addData("x", poseEstimate.getX());
    telemetry.addData("y", poseEstimate.getY());
    telemetry.addData("heading", poseEstimate.getHeading());
    telemetry.update();
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

  private boolean buttonSignal(Button button) {
    return gamepadEx1.isDown(button) || gamepadEx2.isDown(button);
  }

  private double triggerSignal(Trigger trigger) {
    double in1 = gamepadEx1.getTrigger(trigger),
        in2 = gamepadEx2.getTrigger(trigger);
    return Math.max(in1, in2);
  }

  private Vector2d stickSignal(Direction side) {
    Function<GamepadEx, Vector2d> toCoords = pad ->
      side == Direction.LEFT ? new Vector2d(pad.getLeftX(), pad.getLeftY()) :
          new Vector2d(pad.getRightX(), pad.getRightY());

    Vector2d v1 = toCoords.apply(gamepadEx1),
        v2 = toCoords.apply(gamepadEx2);

    return v1.magnitude() > 0.02 ? v1 : v2;
  }

  private boolean justPressed(Button button) {
    return gamepadEx1.wasJustPressed(button) || gamepadEx2.wasJustPressed(button);
  }

  void updateTelemetry() {
    telemetry.addData("Left Joystick (Strafing) X-Val: ", gamepad1.left_stick_x);
    telemetry.addData("Right Joystick (Turning) X-Val: ", gamepad1.right_stick_x);
    telemetry.addData("Left Joystick (Moving) Y-Val: ", gamepad1.left_stick_y);
    telemetry.addData("Cycle rate: ", cycle);
    FtcDashboard.getInstance().getTelemetry().update();
  }
}
