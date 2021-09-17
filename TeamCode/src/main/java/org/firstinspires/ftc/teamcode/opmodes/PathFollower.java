package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.Map;
import kotlin.Unit;
import kotlin.jvm.functions.Function0;
import org.firstinspires.ftc.teamcode.auto.TeleOpPaths;
import org.firstinspires.ftc.teamcode.auto.TeleOpPaths.TeleOpPathElement;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TemplateState;

public class PathFollower {//May or may not be used depending on the game
  private Bot bot = Bot.getInstance();
  private RRMecanumDrive drive = bot.roadRunner;
  private OpMode opmode;
  private TeleOpPaths paths;
  public boolean doingAction = false;

  public PathFollower(OpMode opmode){
    this.opmode = opmode;
    this.paths = new TeleOpPaths(this.opmode);
  }

  private void followTrajectory(TemplateState state, double percent, Trajectory trajectory){
    doingAction = false;
    drive.followTrajectoryAsync(
        drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToSplineHeading(trajectory.get(Math.min(percent, 100) / 100 * trajectory.duration()))
            .build());
  }

  private void followAction(TemplateState state, Function0<Unit> runner){
    if(!doingAction) {
      doingAction = true;
      runner.invoke();
    }
  }


  public void followPath(TemplateState state, int percent, int part){
    TeleOpPathElement element = getElement(state, part);
    if(element instanceof TeleOpPathElement.Path){
      followTrajectory(state, percent, ((TeleOpPathElement.Path)element).getTrajectory());
    }else{
      followAction(state, ((TeleOpPathElement.Action)element).getRunner());
    }
  }

  public boolean isTrajectory(TemplateState state, int part){
    return getElement(state, part) instanceof TeleOpPathElement.Path;
  }

  private TeleOpPaths.TeleOpPathElement getElement(TemplateState state, int part){
    return paths.getTrajectory(state, part);
  }

  public Map<TemplateState, Integer> getPathsInfo(){
    return paths.getTrajectoryListInfo();
  }

  public void stop(){
    drive.mode = Mode.IDLE;
    drive.setDriveSignal(new DriveSignal());
    bot.reset();
  }
}
