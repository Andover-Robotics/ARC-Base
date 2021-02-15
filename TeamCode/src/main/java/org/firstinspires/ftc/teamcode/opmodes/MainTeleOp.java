package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {
  double cycle = 0;
  double prevRead = 0;
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
  private RingMode ringMode = RingMode.OFF;

  /*
    Guidance:
    - Should use module classes from the module package
    - Should make a "Bot" class
    - Should use MecanumDrive from FTCLib; we don't implement mecanum ourselves
   */
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
    driveSpeed = (1 - 0.4 * (gamepad1.left_trigger + gamepad1.right_trigger));
    bot.drive.driveRobotCentric(gamepad1.left_stick_x * driveSpeed,
        -gamepad1.left_stick_y * driveSpeed,
        gamepad1.left_bumper || gamepad1.right_bumper ? 0 : gamepad1.right_stick_x * driveSpeed);
//        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

//    D-pad			Move (Strafing at 100%) optional

//    A 			N/A
//    B 			N/A
//    X 			Line up with tower goal automatically
//    Y 			Feed single ring to shooter
//    if(gamepadEx2.wasJustPressed(Button.X)){
    // TODO line up with tower goal
//    }

//
//Controller 2	(Robot tools)
//
//    Left joystick		Rotate useless servo with decoration on it
//    Right joystick		N/A
//
//    D-pad			Up: Raise wobble arm at constant speed; Down: Lower
//    wobble arm at constant speed;

//    if (gamepad2.dpad_right) {
//      bot.wobbleClaw.raiseArm();
//    } else if (gamepad2.dpad_left) {
//      bot.wobbleClaw.lowerArm();
//    } else {
//      bot.wobbleClaw.stopArm();
//    }
    bot.wobbleClaw.rotateArm(-gamepad2.right_stick_y);

    //    A			Toggle intake; toggle shooter between idle and shooting
//    B			Toggle Wobble Goal Claw
//    X			stop
//    Y			Feed single ring to shooter

    if ((gamepadEx1.getButton(Button.X) || gamepadEx2.getButton(Button.X)) && ringMode != RingMode.OFF) {
      ringMode = RingMode.OFF;
      bot.intake.stop();
      bot.shooter.turnOff();
      bot.wobbleClaw.stopArm();

    } else if (gamepadEx2.wasJustPressed(Button.A) || gamepadEx1.wasJustPressed(Button.A)) {
      ringMode = ringMode == RingMode.INTAKE ? RingMode.SHOOT : RingMode.INTAKE;
    }

    if (gamepad2.right_stick_button || gamepad1.right_stick_button) {
      bot.intake.spit();
    } else if (ringMode == RingMode.INTAKE) {
      bot.intake.run();
      bot.shooter.runIdleSpeed();
    } else if (ringMode == RingMode.SHOOT) {
      bot.intake.stop();
      bot.shooter.runShootingSpeed();
    }

    if (toggleButtonReaders.get("g2b").wasJustReleased()) {
      if (toggleButtonReaders.get("g2b").getState()) {
        bot.wobbleClaw.open();
      } else {
        bot.wobbleClaw.close();
      }
    }

    if (gamepadEx1.wasJustPressed(Button.Y) || gamepadEx2.wasJustPressed(Button.Y)) {
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

  void updateTelemetry() {
    telemetry.addData("Left Joystick (Strafing) X-Val: ", gamepad1.left_stick_x);
    telemetry.addData("Right Joystick (Turning) X-Val: ", gamepad1.right_stick_x);
    telemetry.addData("Left Joystick (Moving) Y-Val: ", gamepad1.left_stick_y);
    telemetry.addData("Cycle rate: ", cycle);
    FtcDashboard.getInstance().getTelemetry().update();
  }
}
