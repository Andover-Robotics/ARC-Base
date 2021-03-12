package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Bot;

@Autonomous(name="DriveWeightedPower", group="Experimental")
public class DriveWeightedPower extends LinearOpMode {
  private Bot bot;
  @Override
  public void runOpMode() throws InterruptedException {
    bot = Bot.getInstance(this);
    waitForStart();

    bot.roadRunner.setWeightedDrivePower(new Pose2d(0, -0.2, 0));
    sleep(500);
    bot.roadRunner.setWeightedDrivePower(new Pose2d());
  }
}
