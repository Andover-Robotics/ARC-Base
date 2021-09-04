package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GlobalVars;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.util.TimingScheduler;
import java.time.temporal.TemporalAccessor;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;
  private PathFollower follower;
  private boolean isManual = true;
  private int percent = 0, part = 0;


  private double fieldCentricOffset = -90.0;
  public enum TemplateState{
    INTAKE(0.5),
    TRANSPORT(0.5),
    OUTTAKE(0.5);

    public final double progressRate;

    TemplateState(double progressRate){this.progressRate = progressRate;}
  }

  Map<TemplateState, Map<Button, TemplateState>> stateMap = new StateMap().getStateMap();

  public TemplateState state = TemplateState.INTAKE;


  void subInit() {
    //TODO: initialize subsystems not initialized in bot constructor
    timingScheduler = new TimingScheduler(this);
    follower = new PathFollower(this);
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement =================================================================================================
    if(justPressed(Button.START)){
      isManual = !isManual;
    }
    if(justPressed(Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
    }

    if(isManual) {
      drive();
    }else{
      followPath();
    }




    //TODO: insert actual teleop stuff here
    //example
    if(justPressed(Button.RIGHT_BUMPER)){
      GlobalVars.slideStage = 5;
    }



    /*//TODO: make control scheme
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
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
  }






  private void drive(){//Driving ===================================================================================
    driveSpeed = 1;//TODO: change depending on mode

    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
            - fieldCentricOffset;
    Vector2d driveVector = stickSignal(Direction.LEFT),
        turnVector = new Vector2d(
            stickSignal(Direction.RIGHT).getX() * Math.abs(stickSignal(Direction.RIGHT).getX()),
            0);
    if (bot.roadRunner.mode == Mode.IDLE) {
      if (centricity)//epic java syntax
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
    if (buttonSignal(Button.BACK)) {
      fieldCentricOffset = bot.imu.getAngularOrientation()
          .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
  }

  private void followPath(){//Path following ===================================================================================
    percent += stickSignal(Direction.LEFT).getY() * state.progressRate;
    percent = Math.max(0, Math.min(100, percent));

    for(Entry<Button, TemplateState> pair : stateMap.get(state).entrySet()){
      if(justPressed(pair.getKey())){
        state = pair.getValue();
        percent = 0;
      }
    }

    if(justPressed(Button.LEFT_STICK_BUTTON) || percent >= 100){
      percent = 0;
      part += 1;
      if(part > follower.getPathsInfo().get(state)){
        part = 0;
        //TODO: add automatic changer?
      }
    }

    follower.followPath(state, percent, part);

    if(!follower.isTrajectory(state, part)){
      drive();
    }
  }

  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
