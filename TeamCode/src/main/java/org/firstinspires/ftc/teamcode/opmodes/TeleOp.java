package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import java.util.concurrent.locks.Lock;

public class TeleOp extends OpMode {
  private Bot bot;
  private GamepadEx gamepadEx1, gamepadEx2;
  //button reader syntax
  // (g1 or g2)  (a, b, lt, lb, etc)
  private ToggleButtonReader g2a, g2b;
  private double driveSpeed;
  enum ShootMode{
    POWERSHOT,
    TOWERGOAL
  }
  enum TowerMode{
    HIGH,
    MIDDLE,
    LOW
  }

  ShootMode shootState = ShootMode.TOWERGOAL;
TowerMode towerState = TowerMode.HIGH;

  /*
    Guidance:
    - Should use module classes from the module package
    - Should make a "Bot" class
    - Should use MecanumDrive from FTCLib; we don't implement mecanum ourselves
   */

  @Override
  public void init() {
    bot = new Bot(this);
    gamepadInit();

    telemetry.addLine("Initialized successfully!");
    telemetry.update();
  }

  private void gamepadInit() {
    gamepadEx1 = new GamepadEx(gamepad1);
    gamepadEx2 = new GamepadEx(gamepad2);
    g2a = new ToggleButtonReader(gamepadEx2, Button.A);
    g2b = new ToggleButtonReader(gamepadEx2, Button.B);
  }

  @Override
  public void loop() {

    updateTelemetry();

    updateButtons();

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
        bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);//may have to switch angle

//    D-pad			Move (Strafing at 100%) optional

//    A 			N/A
//    B 			N/A
//    X 			Line up with tower goal automatically
//    Y 			Feed single ring to shooter
//    if(gamepadEx2.wasJustPressed(Button.X)){
    // TODO line up with tower goal
//    }

    if(gamepadEx1.wasJustPressed(Button.Y))
      bot.shooter.feedRing();

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
//    A			Toggle intake; toggle shooter between idle and shooting
//    B			Toggle Wobble Goal Claw
//    X			N/A
//    Y			Feed single ring to shooter
    if(g2a.wasJustPressed())
      if (g2a.getState())
        bot.intake.run();
      else
        bot.intake.stop();

    if(g2b.wasJustPressed())
      if(g2a.getState())
        bot.wobbleClaw.open();
      else
        bot.wobbleClaw.close();

    if(gamepadEx2.wasJustPressed(Button.Y))
      bot.shooter.feedRing();
//
//    Start			N/A
//    Select			N/A
//
//    L. Bumper		Toggle Power Shot mode
//    R. Bumper		Cycle between High, Mid, and Low Goal
//    L. Trigger		N/A
//    R. Trigger		N/A
    if(gamepadEx2.wasJustPressed(Button.LEFT_BUMPER))
      if(shootState == ShootMode.POWERSHOT)
        shootState = ShootMode.TOWERGOAL;
      else
        shootState = ShootMode.POWERSHOT;

    if(gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER))
      switch(towerState){
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

    CommandScheduler.getInstance().run();
  }

  private void updateTelemetry() {
    telemetry.addData("Left Joystick (Strafing) X-Val: ", gamepad1.left_stick_x);
    telemetry.addData("Right Joystick (Turning) X-Val: ", gamepad1.right_stick_x);
    telemetry.addData("Left Joystick (Moving) Y-Val: ", gamepad1.left_stick_y);
  }

  private void updateButtons() {
    g2a.readValue();
    g2b.readValue();
    gamepadEx1.readButtons();
  }
}
