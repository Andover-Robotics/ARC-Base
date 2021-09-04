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
import org.firstinspires.ftc.teamcode.util.TimingScheduler;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;



  private double fieldCentricOffset = -90.0;

  private PathFollower follower;


  void subInit() {
    //TODO: initialize subsystems not initialized in bot constructor
    timingScheduler = new TimingScheduler(this);
    follower = new PathFollower(bot.roadRunner);
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

//Driving =================================================================================================
    driveSpeed = 1;//TODO: change depending on mode


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


    //TODO: insert actual teleop stuff here



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
    L:      R:
     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
  }



  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
