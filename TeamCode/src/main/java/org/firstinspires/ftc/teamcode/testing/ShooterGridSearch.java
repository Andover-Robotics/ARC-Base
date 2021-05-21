package org.firstinspires.ftc.teamcode.testing;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.FileNotFoundException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.stream.IntStream;
import org.firstinspires.ftc.teamcode.hardware.Shooter;

@TeleOp(
    name = "Shooter Grid Search",
    group = "Experimental"
)
public final class ShooterGridSearch extends OpMode {
  private DcMotorEx flywheel;
  private Servo feeder;
  private VoltageSensor volts;

  private FtcDashboard dash;
  private GamepadEx pad;
  private StreamingDataWriter writer;

  private final double[] gridShootRpm = {2800, 3100, 3400, 3700};
  private final double[] gridP = {19, 21, 23};
  private final double[] gridI = {0, 0.1};
  private final double[] gridD = {0.05, 0.1, 0.15};
  private final int[] gridFeedDuration = {100, 110, 120};
  private int feedGapDuration = 200;
  private int feedDuration = 100;
  private int ordinal = 0;

  // F - feedDuration - B - feedGapDuration - repeat
  private Double feederStartTime = null;

  public void init() {
    setupFields();
    try {
      String dateStr = new SimpleDateFormat("yyyy-MM-dd_HH-mm").format(new Date());
      writer = new StreamingDataWriter(20, () -> {
        PIDFCoefficients pid = flywheel.getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
        return new double[] {
            getRuntime() - (feederStartTime == null ? 100000 : feederStartTime),
            currentTargetRpm(),
            feedDuration,
            feedGapDuration,
            flywheel.getVelocity() * 60.0 / 28.0,
            volts.getVoltage(),
            pid.p,
            pid.i,
            pid.d,
            pid.f,
            ordinal
        };
      }, "/sdcard/FIRST/shooter-grid_" + dateStr + ".csv");
      writer.writeLine("time,target,feed,gap,velocity,voltage,p,i,d,f,ord");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
      stop();
    }
    feederStartTime = getRuntime();
    flywheel.setVelocity(3400.0 / 60.0 * 28.0);
    flywheel.setVelocityPIDFCoefficients(0, 0, 0, 11.6);
  }


  public void init_loop() {
    if (getRuntime() - feederStartTime > 0.2) {
      if (gamepad1.dpad_up) ordinal++;
      feederStartTime = getRuntime();
    }

    telemetry.addData("flywheel rpm", flywheel.getVelocity() * 60.0 / 28.0);
    telemetry.addData("ordinal", ordinal);
  }

  private void setupFields() {
    this.flywheel = this.hardwareMap.get(DcMotorEx.class, "shooter");
    flywheel.setMode(RunMode.RUN_USING_ENCODER);
    flywheel.setDirection(Direction.REVERSE);
    this.feeder = this.hardwareMap.servo.get("magazine");
    volts = this.hardwareMap.voltageSensor.iterator().next();
    this.dash = FtcDashboard.getInstance();
    telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
    pad = new GamepadEx(gamepad1);
  }

  public void start() {
    writer.startStreaming();
    writer.pause();
    feederStartTime = null;
    flywheel.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, currentPidf());
    applyShootRpm();
  }

  public void loop() {

    if (pad.wasJustPressed(Y)) {
      feederStartTime = getRuntime();
      writer.resume();
    }
    if (feederStartTime != null) {
      feederLoop();
    }

    telemetry.addData("flywheel rpm", flywheel.getVelocity() * 60.0 / 28.0);
    telemetry.addData("flywheel target rpm", currentTargetRpm());
    telemetry.addData("feeder duration", feedGapDuration);
    telemetry.addData("feeder up duration", feedDuration);
    telemetry.addData("pidf", flywheel.getPIDFCoefficients(RunMode.RUN_USING_ENCODER));
    telemetry.addData("paused?", writer.paused());
    telemetry.addData("ordinal", ordinal);

    pad.readButtons();
  }

  public void stop() {
    writer.stopStreaming();
  }

  public final void applyShootRpm() {
    flywheel.setVelocity(28.0 * currentTargetRpm() / 60.0);
  }

  public final void feederLoop() {
    double dt = (getRuntime() - feederStartTime) * 1000;
    if (dt / (feedGapDuration + feedDuration) > 3) {
      if (dt / (feedGapDuration + feedDuration) > 3 + 0.4) {
        feederStartTime = null;
        ordinal++;
        applyShootRpm();
        feedDuration = currentFeedDuration();
        flywheel.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, currentPidf());
        writer.pause();
      }
    } else {
      double iterDt = dt % (feedGapDuration + feedDuration);
      feeder.setPosition(iterDt >= feedDuration ? Shooter.magazineBackward : Shooter.magazineForward);
    }
  }

  private double currentTargetRpm() {
    return gridShootRpm[ordinal % gridShootRpm.length];
  }

  private PIDFCoefficients currentPidf() {
     double p = gridP[(ordinal / gridShootRpm.length) % gridP.length],
         i = gridI[(ordinal / gridShootRpm.length / gridP.length / gridFeedDuration.length / gridD.length) % gridI.length],
         d = gridD[(ordinal / gridShootRpm.length / gridP.length / gridFeedDuration.length) % gridD.length],
         f = flywheel.getPIDFCoefficients(RunMode.RUN_USING_ENCODER).f;
     return new PIDFCoefficients(p, i, d, f);
  }

  private int currentFeedDuration() {
    return gridFeedDuration[(ordinal / gridShootRpm.length / gridP.length) % gridFeedDuration.length];
  }
}
