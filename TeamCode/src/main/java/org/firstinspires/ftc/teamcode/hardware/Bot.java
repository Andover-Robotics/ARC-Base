package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;

public class Bot {
  // in TeleOp and Autonomous we should be able to call "new Bot(this)"
  // bot.intake.run(), bot.shooter.spinUp
  private static Bot instance;

  public final Intake intake;
  public final Shooter shooter;
  public final WobbleClaw wobbleClaw;
  public final MecanumDrive drive;
  public final RRMecanumDrive roadRunner;
  public final BNO055IMU imu;
  public OpMode opMode;

  /** Get the current Bot instance from somewhere other than an OpMode */
public static Bot getInstance() {
    if (instance == null) {
      throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
    }
    return instance;
  }

  public static Bot getInstance(OpMode opMode) {
    if (instance == null) {
      return instance = new Bot(opMode);
    }
    instance.opMode = opMode;
    return instance;
  }

  private Bot(OpMode opMode){
    this.opMode = opMode;
    this.intake = new Intake(opMode);
    this.shooter = new Shooter(opMode);
    this.wobbleClaw = new WobbleClaw(opMode);
    this.roadRunner = new RRMecanumDrive(opMode.hardwareMap);
    imu = roadRunner.imu;
    this.drive = new MecanumDrive(false,
        new MotorEx(opMode.hardwareMap, "motorFL"),
        new MotorEx(opMode.hardwareMap, "motorFR"),
        new MotorEx(opMode.hardwareMap, "motorBL"),
        new MotorEx(opMode.hardwareMap, "motorBR"));
  }

//  private void initializeImu() {
//    final Parameters params = new Parameters();
//    params.angleUnit = AngleUnit.RADIANS;
//    imu.initialize(params);
//  }
}
