package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.Mode;

public class PathFollower {//May or may not be used depending on the game
  private RRMecanumDrive drive;

  public PathFollower(RRMecanumDrive drive){
    this.drive = drive;
  }


  //TODO: not for template - find a way to make followTrajectory allow for multiple
  // AutoPathElements and to follow separate paths
  public void followTrajectory(double percent, Trajectory trajectory){
    drive.followTrajectoryAsync(
        drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToSplineHeading(trajectory.get(percent * trajectory.duration()))
            .build());
  }

  public void stop(){
    drive.mode = Mode.IDLE;
    drive.setDriveSignal(new DriveSignal());
  }
}
