package org.firstinspires.ftc.teamcode.demo

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.hardware.WobbleClaw
import org.firstinspires.ftc.teamcode.testing.StreamingDataWriter
import java.util.*
import kotlin.concurrent.scheduleAtFixedRate

@TeleOp(name = "Run to position")
class RunToPositionDemo : OpMode() {
    lateinit var motor: DcMotorEx
    lateinit var claw: Servo

    val writer = StreamingDataWriter(20, {
        val pidf = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
        doubleArrayOf(
                motor.power,
                motor.currentPosition.toDouble(),
                motor.targetPosition.toDouble(),
                motor.getCurrent(CurrentUnit.MILLIAMPS),
                pidf.p,
                pidf.i,
                pidf.d,
                pidf.f)
    }, "/sdcard/FIRST/rtp-${System.currentTimeMillis() % 1000}.csv")

    val timer = Timer()

    var setPower = 0.2
    var setP = 0.0
    var setI = 0.0
    var setD = 0.0
    var setF = 0.0

    override fun init() {
        motor = hardwareMap.get(DcMotorEx::class.java, "wobbleArm")
        claw = hardwareMap.servo.get("claw")
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = 0.0
        val coeff = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
        setP = coeff.p
        setI = coeff.i
        setD = coeff.d
        setF = coeff.f
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    }

    override fun start() {
        writer.startStreaming()
        timer.scheduleAtFixedRate(0, 300) {
            gamepad1.run {
                if (dpad_up) motor.targetPosition += 50
                if (dpad_down) motor.targetPosition -= 50
                if (y) setP += 0.5
                if (a) setP -= 0.5
                if (left_stick_y < -0.5) setI += 0.5
                if (left_stick_y > 0.5) setI -= 0.5
                if (right_stick_y < -0.5) setD += 0.5
                if (right_stick_y > 0.5) setD -= 0.5
                if (left_bumper) setF += 1
                if (right_bumper) setF -= 1
                if (dpad_left) setPower -= 0.05
                if (dpad_right) setPower += 0.05
                if (left_stick_button) claw.position =
                        if (claw.position == WobbleClaw.clawClosedPos) WobbleClaw.clawOpenPos else WobbleClaw.clawClosedPos
            }
        }
    }

    override fun loop() {
        // Apply powers
        motor.power = if (gamepad1.x) setPower else 0.0
        motor.setVelocityPIDFCoefficients(setP, setI, setD, setF)

        telemetry.run {
            addData("power", setPower)
            addData("current", motor.currentPosition)
            addData("target", motor.targetPosition)
            addData("pidf", "%.3f %.3f %.3f %.3f", setP, setI, setD, setF)
            addData("current", motor.getCurrent(CurrentUnit.MILLIAMPS))
            addData("<i>Controls</i>", "<ul><li>Dpad Up/Down: Target</li><li>Y/A: P</li>" +
                    "<li>Left stick: I</li><li>Right stick: D</li><li>Bumpers: F</li><li>Dpad Left/Right: Power</li></ul>")
        }

    }

    override fun stop() {
        writer.stopStreaming()
        timer.cancel()
    }

}