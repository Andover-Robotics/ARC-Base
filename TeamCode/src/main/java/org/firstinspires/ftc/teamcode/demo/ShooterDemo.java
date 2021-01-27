package org.firstinspires.ftc.teamcode.demo;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter demo", group = "Experimental")
public class ShooterDemo extends OpMode {
  boolean on = false;
  Motor motor;

  @Override
  public void init() {
    motor = new Motor(hardwareMap, "arm", GoBILDA.RPM_1150);
//    motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    motor.setRunMode(RunMode.RawPower);
  }

  @Override
  public void init_loop() {
  }

  @Override
  public void loop() {
    if(gamepad1.a){
      on = true;
    }
    if(gamepad1.b){
      on = false;
    }
    if(on){
      motor.set(1);
    }else{
      motor.set(0);
    }
  }
}
