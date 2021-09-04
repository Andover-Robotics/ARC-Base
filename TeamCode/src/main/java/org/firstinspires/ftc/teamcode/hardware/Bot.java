package org.firstinspires.ftc.teamcode.hardware;

import android.util.Pair;
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
import org.openftc.revextensions2.ExpansionHubEx;

public class Bot {
  // in TeleOp and Autonomous we should be able to call "new Bot(this)"
  // bot.intake.run(), bot.shooter.spinUp
  public static Bot instance;


  //TODO: Declare subsystems here
  //example
  //public final Intake intake;


  //required subsystems
  public final MecanumDrive drive;
  public final RRMecanumDrive roadRunner;
  public final BNO055IMU imu;
  public Pair<ExpansionHubEx, ExpansionHubEx> hubs = null;
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
    enableAutoBulkRead();
    try {
      this.hubs = Pair.create(opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"),
          opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"));
      hubs.first.setPhoneChargeEnabled(true);
      hubs.second.setPhoneChargeEnabled(true);
    } catch (Exception e) {
      // Avoid catastrophic errors if RevExtensions don't behave as expected. Limited trust of stability
      e.printStackTrace();
    }

    //TODO: initialize subsystems
    //example
    //this.intake = new Intake(opMode);



    //required subsystems
    this.drive = new MecanumDrive(false,
        new MotorEx(opMode.hardwareMap, "motorFL"),
        new MotorEx(opMode.hardwareMap, "motorFR"),
        new MotorEx(opMode.hardwareMap, "motorBL"),
        new MotorEx(opMode.hardwareMap, "motorBR"));
    this.roadRunner = new RRMecanumDrive(opMode.hardwareMap);
    imu = roadRunner.imu;
  }

//  private void initializeImu() {
//    final Parameters params = new Parameters();
//    params.angleUnit = AngleUnit.RADIANS;
//    imu.initialize(params);
//  }

  private void enableAutoBulkRead() {
    for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
      mod.setBulkCachingMode(BulkCachingMode.AUTO);
    }
  }
}
