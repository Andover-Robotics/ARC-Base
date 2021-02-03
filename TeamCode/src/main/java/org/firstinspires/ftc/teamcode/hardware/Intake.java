package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
  public static double intakeSpeed = 0.4;

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

  private CRServo convBelt;
  private Motor actuator;

  public Intake(OpMode opMode) {
    convBelt = opMode.hardwareMap.crservo.get("convBelt");
    actuator = new Motor(opMode.hardwareMap, "intake", GoBILDA.RPM_435);
    actuator.setInverted(true);
  }

  public void run() {
    actuator.set(intakeSpeed);
    convBelt.setPower(1);
  }

  public void stop() {
    actuator.stopMotor();
    convBelt.setPower(0);
  }
}
