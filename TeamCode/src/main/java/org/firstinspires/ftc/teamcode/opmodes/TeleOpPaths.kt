package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.GlobalVars
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.Bot
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TemplateState.*
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt

class TeleOpPaths(val opMode/*unused? keep for action cases*/: OpMode) {

    sealed class TeleOpPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory): TeleOpPathElement(name)
        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit): TeleOpPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): TeleOpPathElement.Path{
        lastPosition = trajectory.end()
        return TeleOpPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): TeleOpPathElement.Action{
        return TeleOpPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit): MarkerCallback {
        override fun onMarkerReached() = func()
    }

    private fun turn(from: Double, to: Double): TeleOpPathElement.Action {
        return TeleOpPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                "to ${Math.toDegrees(to).roundToInt()}deg") {
            bot.roadRunner.turn(to - from)
        }
    }

    //TODO: Insert pose/vector vals here

    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )

    val startPose = Pose2d(-48.0 - 24 + 9, -34.0, PI)

    //TODO: insert action vals here

    //                                                                  =======================================================

    //example
    //private val shootRings = AutoPathElement.Action("Shoot 3 rings") {
    //        bot.shooter.shootRings(opMode, 3, 0.8)
    //        bot.shooter.turnOff()
    //        Thread.sleep(1000)
    //    }


    //TODO: Make Trajectories

    //                                                                              ====================================================
    private val trajectorySets: Map<MainTeleOp.TemplateState, List<TeleOpPathElement>> = mapOf(
            //use !! when accessing maps ie: dropSecondWobble[0]!!
            //example
            INTAKE to run{
                val specificPose = Pose2d()
                listOf(
                        makePath("part 1",
                                drive.trajectoryBuilder(startPose)
                                        .build()),

                        makeAction("do something") {
                            Thread.sleep(GlobalVars.slideStage)
                        },

                        makePath("part 2",
                                drive.trajectoryBuilder(lastPosition)
                                        .forward(10.0)
                                        .build())
                )
            }
//
    )


    fun getTrajectories(a: MainTeleOp.TemplateState): List<TeleOpPathElement>{
        return trajectorySets[a]!!
    }

    fun getTrajectory(set: MainTeleOp.TemplateState, part: Int): TeleOpPathElement{//will have to add logic for actions in teleop
        return trajectorySets[set]!!.get(part)
    }
    fun getTrajectoryListInfo(): Map<MainTeleOp.TemplateState, Int>{//return number of trajectories per set
        return trajectorySets.mapValues{it.value.size}
    }



}