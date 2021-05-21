package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.Function;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.util.TimingScheduler;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {
  private double cycle = 0;
  private double prevRead = 0;
  private double fieldCentricOffset = -90.0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;

  enum ShootMode {
    POWER_SHOT( 3200),
    TOWER_GOAL( 3750);

    public final int rpm;

    ShootMode(int rpm) {
      this.rpm = rpm;
    }
  }

  enum RingMode {
    INTAKE,
    SHOOT,
    OFF
  }

  private ShootMode shootState = ShootMode.TOWER_GOAL;
  private RingMode ringMode = RingMode.INTAKE;
  private ShootingAlignment aligner;
  private boolean clawOpen = true;

  void subInit() {
    bot.shooter = new Shooter(this);
    timingScheduler = new TimingScheduler(this);
    aligner = new ShootingAlignment(bot.roadRunner);
  }

  void buttonsInit() {

  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

//Driving =================================================================================================
    driveSpeed = (ringMode == RingMode.SHOOT ? 0.5 : 1) * (1 - 0.33 * (triggerSignal(Trigger.LEFT_TRIGGER) + triggerSignal(Trigger.RIGHT_TRIGGER)));
    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle - fieldCentricOffset;
    Vector2d driveVector = stickSignal(Direction.LEFT),
        turnVector = new Vector2d(
            stickSignal(Direction.RIGHT).getX() * Math.abs(stickSignal(Direction.RIGHT).getX()), 0);
    if(bot.roadRunner.mode == Mode.IDLE) {
      if(centricity)//epic java syntax
        bot.drive.driveFieldCentric(
          driveVector.getX() * driveSpeed,
          driveVector.getY() * driveSpeed,
          turnVector.getX() * driveSpeed,
          gyroAngle);
      else
        bot.drive.driveRobotCentric(
            -driveVector.getX() * driveSpeed,
            -driveVector.getY() * driveSpeed,
            turnVector.getX() * driveSpeed
        );
    }
    if (buttonSignal(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset = bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }






//Toggle modes====================================================================================
    if(justPressed(Button.X)){
      aligner.runToPositionToggle();
    } else if (justPressed(Button.A)) {
      ringMode = ringMode == RingMode.INTAKE ? RingMode.SHOOT : RingMode.INTAKE;
    } else if (justPressed(Button.B)){
      shootState = shootState == ShootMode.POWER_SHOT ? ShootMode.TOWER_GOAL : ShootMode.POWER_SHOT;
    }

    //Do intake/shooter====================================================================================
    if (buttonSignal(Button.RIGHT_BUMPER)) {
      bot.intake.spit();
    } else if (ringMode == RingMode.INTAKE) {
      bot.intake.run();
      bot.shooter.runIdleSpeed();
    } else if (ringMode == RingMode.SHOOT) {
      if(buttonSignal(Button.LEFT_BUMPER)){
        bot.intake.run(0.6);
      }else {
        bot.intake.stop();
      }

      bot.shooter.runShootingSpeed(shootState.rpm);
    }


    //Wobble claw====================================================================================
    if (buttonSignal(Button.DPAD_UP)) {
      bot.wobbleClaw.close();
      timingScheduler.defer(0.5, bot.wobbleClaw::raiseArm);
    } else if (buttonSignal(Button.DPAD_DOWN)) {
      bot.wobbleClaw.lowerArm();
      timingScheduler.defer(0.5, bot.wobbleClaw::open);
    } else if (justPressed(Button.DPAD_LEFT)) {
      bot.wobbleClaw.lowerArmToDropInEndgame();
      timingScheduler.defer(0.5, bot.wobbleClaw::open);
    }


//    //Blockers====================================================================================
//    if (buttonSignal(Button.DPAD_RIGHT)) {//TODO? bind this to ringMode?
//      bot.intake.closeBlockers();
//    } else {
//      bot.intake.openBlockers();
//    }

    //Shoot====================================================================================
    if (justPressed(Button.Y)) {
      bot.shooter.startFeederLoop(getRuntime());
    } else if (justReleased(Button.Y)) {
      bot.shooter.bringArmBack();
    } else if (buttonSignal(Button.Y)) {
      bot.shooter.feederLoop(getRuntime());
    }

    //Flipper==================================================================================
    if(justPressed(Button.DPAD_RIGHT)){
      bot.shooter.flipUp();
    }else if(justReleased(Button.DPAD_RIGHT)){
      bot.shooter.flipDown();
    }


    //robotvsfieldcentric=================================================================
    if(justPressed(Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
    }

    /*
    Total control scheme(same for both)
    A:switch shoot/intake      B:switch power/tower      X:move to shoot      Y:Shoot
    DPAD
    L:wc drop      D:wc down      U:wc up      R:flipper
    Joystick
    L:movement/reset field centric    R:movement/none
    Trigger L/R: slow
    Bumper
    L:Manual Conveyor      R:Reverse conveyor
     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
    telemetry.addData("shoot state", shootState);
  }


  //Buttons=================================================================================================
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

  private boolean justReleased(Button button){
    return !(gamepadEx1.isDown(button) || gamepadEx2.isDown(button)) && (gamepadEx1.wasJustReleased(button) || gamepadEx2.wasJustReleased(button));
  }

  //Telemetry=================================================================================================
  void updateTelemetry() {
    telemetry.addData("Left Joystick (Strafing) X-Val: ", gamepad1.left_stick_x);
    telemetry.addData("Right Joystick (Turning) X-Val: ", gamepad1.right_stick_x);
    telemetry.addData("Left Joystick (Moving) Y-Val: ", gamepad1.left_stick_y);
    telemetry.addData("Cycle rate: ", cycle);
    telemetry.addData("Ring mode", ringMode);
//    FtcDashboard.getInstance().getTelemetry().update();
  }

  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
