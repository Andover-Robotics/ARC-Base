package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Intake extends SubsystemBase {
  public static class RunIntake extends CommandBase {
    private final Intake intake;

    public RunIntake(Intake intake) {
      this.intake = intake;
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.run();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stop();
    }
  }

  private Motor actuator;

  public Intake(OpMode opMode) {
    actuator = new Motor(opMode.hardwareMap, "intake", GoBILDA.RPM_435);
  }

  public void run() {
    actuator.set(1);
  }

  public void stop() {
    actuator.stopMotor();
  }
}
