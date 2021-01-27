package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Bot {
  // in TeleOp and Autonomous we should be able to call "new Bot(this)"
  // bot.intake.run(), bot.shooter.spinUp

  public final Intake intake;
  public final Shooter shooter;
  public final WobbleClaw wobbleClaw;
  public final MecanumDrive drive;
  public final BNO055IMU imu;

  public Bot(OpMode opMode){
    this.intake = new Intake(opMode);
    this.shooter = new Shooter(opMode);
    this.wobbleClaw = new WobbleClaw(opMode);
    this.drive = new MecanumDrive(true,
        new MotorEx(opMode.hardwareMap, "motorFL", GoBILDA.RPM_312),
        new MotorEx(opMode.hardwareMap, "motorFR", GoBILDA.RPM_312),
        new MotorEx(opMode.hardwareMap, "motorBL", GoBILDA.RPM_312),
        new MotorEx(opMode.hardwareMap, "motorBR", GoBILDA.RPM_312)
    );
    imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
    initializeImu();
  }

  private void initializeImu() {
    final Parameters params = new Parameters();
    params.angleUnit = AngleUnit.RADIANS;
    imu.initialize(params);
  }
}
