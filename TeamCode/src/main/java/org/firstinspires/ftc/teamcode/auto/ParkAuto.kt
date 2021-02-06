package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Bot

@Autonomous(name = "Park Auto")
class ParkAuto: LinearOpMode() {
    override fun runOpMode() {
        Bot.instance = null
        val bot = Bot.getInstance(this)

        waitForStart()

        bot.roadRunner.followTrajectory(bot.roadRunner.trajectoryBuilder(Pose2d())
                .forward(24.0 * 3)
                .build())
    }
}