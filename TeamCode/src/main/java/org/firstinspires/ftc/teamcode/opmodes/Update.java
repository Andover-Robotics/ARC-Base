package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Update extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry.addData("update",  10);
    telemetry.update();
    waitForStart();
  }
}
