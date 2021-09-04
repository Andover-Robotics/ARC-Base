package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

  public PathFollower(OpMode opmode){
    this.opmode = opmode;
    this.paths = new TeleOpPaths(this.opmode);
  }


  //TODO: not for template - find a way to make followTrajectory allow for multiple
  // AutoPathElements and to follow separate paths
  //
  //have 1st driver control trajecotries?
  public void followTrajectory(double percent, Trajectory trajectory){
    drive.followTrajectoryAsync(
        drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToSplineHeading(trajectory.get(percent * trajectory.duration()))
            .build());
  }
// TODO: find way to allow for trajectories and actions

//  public void goToTrajectoryEnd(TemplateState state, int part){
//    TeleOpPathElement element = getElement(state, part);
//    drive.followTrajectoryAsync(
//        drive.trajectoryBuilder(drive.getPoseEstimate())
//            .lineToSplineHeading(.end())
//            .build());
//  }

  private TeleOpPaths.TeleOpPathElement getElement(TemplateState state, int part){
    return paths.getTrajectory(state, part);
  }

  public void stop(){
    drive.mode = Mode.IDLE;
    drive.setDriveSignal(new DriveSignal());
  }
}
