package org.firstinspires.ftc.teamcode.demo

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

@Autonomous(name = "Run to position", group = "Experimental")
class RunToPositionDemo : LinearOpMode() {

    override fun runOpMode() {
        val motor = hardwareMap.dcMotor.get("wobbleArm")
        waitForStart()
        motor.targetPosition = 100
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = 0.4

        while (!isStopRequested && motor.isBusy) {
            telemetry.addData("motor position", motor.currentPosition)
            telemetry.update()
        }

        while (!isStopRequested) {}
    }
}