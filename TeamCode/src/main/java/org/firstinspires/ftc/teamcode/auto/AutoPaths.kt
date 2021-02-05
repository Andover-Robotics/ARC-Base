package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.roundToInt

class AutoPaths(val opMode: LinearOpMode) {

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory): AutoPathElement(name)
        class Action(override val name: String, val runner: () -> Unit): AutoPathElement(name)
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))

    // (-3.36345, -0.0756263), (72-22.75, 0)
    private val startPose = Pose2d(-48.0 - 24 + 9, -34.0, 0.0)
    private val ramIntoStack = Vector2d(-24.0, -24.0 - 12.0)
    private val avoidStack = Vector2d(-24.0, -17.0)
    private val dropFirstWobble = mapOf(
            0 to Pose2d(-0.0756263, -72.0 + 22.75 + 3.36345, (-90.0 + 39.05465).toRadians),
            1 to Pose2d(24 - 2.668, -24 - 11.416 - 0.6, (-90 + 85.87698).toRadians),
            4 to Pose2d(48.0 + 1.4313, -48 + 5.35248, (-90 + 31.4772).toRadians)
    )
    private val dropSecondWobble = mapOf(
            0 to Pose2d(-8.8717, -48.0 - 5.344, (-90.0 + 39.0579).toRadians),
            1 to Pose2d(24.0 - 9.45428, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
            4 to Pose2d(48 - 7.1, -48.0 - 3.0556, (-90.0 + 30.268).toRadians)
    )

    //getting second wobble goal for 1 and 4
    private val secondWobbleLocation = Vector2d(-48.0, -48 - 2.0)
    private val secondWobbleApproachAngle = 160.0.toRadians
    private val secondWobbleApproach = Pose2d(secondWobbleLocation.plus(Vector2d(18.8722, -0.622)), secondWobbleApproachAngle)

    private val dropWobble = AutoPathElement.Action("Drop wobble") {
        bot.wobbleClaw.lowerArm()
        bot.wobbleClaw.waitUntilTargetReached(opMode)
        bot.wobbleClaw.open()
        Thread.sleep(700)
        bot.wobbleClaw.raiseArm()
        bot.wobbleClaw.waitUntilTargetReached(opMode)
    }

    private val pickUpWobble = AutoPathElement.Action("Pick up wobble") {
        bot.wobbleClaw.open()
        bot.wobbleClaw.lowerArm()
        bot.wobbleClaw.waitUntilTargetReached(opMode)
        bot.wobbleClaw.close()
        Thread.sleep(1200)
        bot.wobbleClaw.raiseArm()
        bot.wobbleClaw.waitUntilTargetReached(opMode)
    }

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                "to ${Math.toDegrees(to).roundToInt()}deg") {
            bot.roadRunner.turn(to - from)
        }
    }

    private val trajectorySets: Map<Int, List<AutoPathElement>> = mapOf(
            0 to run {
                val secondWobble = Pose2d(secondWobbleLocation.plus(Vector2d(14.073, 12.58958)), (294.2298 - 90.0).toRadians)
                // intermediate waypoint for 3 point turn
                val intermediate = Pose2d(5.0, -24 - 6.0, 0.0)

                listOf(
                        AutoPathElement.Path("Go drop first wobble",
                                drive.trajectoryBuilder(startPose, startPose.heading)
                                .splineTo(dropFirstWobble[0]!!.vec(), dropFirstWobble[0]!!.heading)
                                //.addDisplacementMarker({ MainTeleOp.testSomething()})
                                .build()),

                        dropWobble,

                        // intermediate point turn
                        AutoPathElement.Path("Make intermediate point turn",
                                drive.trajectoryBuilder(dropFirstWobble[0]!!, dropFirstWobble[0]!!.heading + 180.0.toRadians)
                                .splineTo(intermediate.vec(), intermediate.heading)
                                .build()),

                        // go to get second wobble
                        AutoPathElement.Path("Go get second wobble",
                                drive.trajectoryBuilder(Pose2d(intermediate.vec(), intermediate.heading + Math.PI), intermediate.heading + Math.PI)
                                .splineTo(secondWobble.vec(), secondWobble.heading)
                                .build()),

                        pickUpWobble,

                        //go to deposit second wobble
                        AutoPathElement.Path("Go drop second wobble",
                                drive.trajectoryBuilder(secondWobble)
                                .splineTo(dropSecondWobble[0]!!.vec(), dropSecondWobble[0]!!.heading)
                                .build()),

                        dropWobble,

                        // go park
                        AutoPathElement.Path("Go park",
                                drive.trajectoryBuilder(dropSecondWobble[0]!!, dropSecondWobble[0]!!.heading + Math.PI)
                                .splineToSplineHeading(Pose2d(11.0, -25.0, 0.0), 0.0)
                                .build())

                )
            },
            1 to run {

//                val pose2 = Pose2d(20.0, -35.0, toRadians(175.0))
//                val pose3 = Pose2d(secondWobbleLocation.x, secondWobbleLocation.y, secondWobbleApproachAngle + Math.PI)
//                val ringLoc = Vector2d(-20.0, -35.0)//looks good as per visualizer
//                val intermediate = Pose2d(-11.0, -53.0, (178.0).toRadians)
//                val beforeIntermediate = Pose2d(-6.7, -38.0, 180.0.toRadians)
//                val intermediateReturn = Pose2d(-7.68, -39.36, 90.0.toRadians)

                listOf(
                        AutoPathElement.Path("Go drop first wobble",
                                drive.trajectoryBuilder(startPose)
                                .splineTo(ramIntoStack, 0.0)
                                .splineTo(dropFirstWobble[1]!!.vec(), dropFirstWobble[1]!!.heading)
                                .build()),

                        dropWobble,
                        turn(from = dropFirstWobble[1]!!.heading, to = 200.0.toRadians),

                        AutoPathElement.Path("Go pick up second wobble",
                                drive.trajectoryBuilder(dropFirstWobble[1]!!.copy(heading = 200.0.toRadians))
                                .splineTo(secondWobbleApproach.vec(), secondWobbleApproachAngle)
                                .build()),

                        pickUpWobble,

                        AutoPathElement.Path("Go drop second wobble",
                                drive.trajectoryBuilder(secondWobbleApproach)
                                .splineTo(Vector2d(-27.84, -45.6), -(15.0.toRadians))
                                .splineTo(dropSecondWobble[1]!!.vec(), dropSecondWobble[1]!!.heading)
                                .build()),

                        dropWobble
                )
            },
            4 to run {
                val intermediateTurn = Pose2d(61.0, -24.0, -(170.0.toRadians))

                listOf(
                        // go to drop first wobble
                        AutoPathElement.Path("Go drop first wobble",
                                drive.trajectoryBuilder(startPose, startPose.heading)
                                .splineTo(avoidStack, 0.0)
                                .splineTo(dropFirstWobble[4]!!.vec(), dropFirstWobble[4]!!.heading)
                                .build()),

                        dropWobble,

                        // intermediate point turn
                        AutoPathElement.Path("Make intermediate point turn",
                                drive.trajectoryBuilder(dropFirstWobble[4]!!, dropFirstWobble[4]!!.heading + Math.PI)
                                .splineTo(intermediateTurn.vec(), intermediateTurn.heading + Math.PI)
                                .build()),
                        // go to get second wobble
                        AutoPathElement.Path("Go get second wobble",
                                drive.trajectoryBuilder(intermediateTurn)
                                .splineTo(secondWobbleApproach.vec(), secondWobbleApproach.heading)
                                .build()),

                        pickUpWobble,

                        //go to deposit second wobble
                        AutoPathElement.Path("Go drop second wobble",
                                drive.trajectoryBuilder(secondWobbleApproach)
                                .splineTo(avoidStack, 0.0)
                                .splineTo(dropSecondWobble[4]!!.vec(), dropSecondWobble[4]!!.heading)
                                .build()),

                        dropWobble,

                        // go park
                        AutoPathElement.Path("Go park",
                                drive.trajectoryBuilder(dropSecondWobble[4]!!, dropSecondWobble[4]!!.heading + Math.PI)
                                .splineTo(Vector2d(11.5, -42.24), Math.PI)
                                .build())
                )
            }
//            4 to listOf(
//                    // go to drop first wobble
//                    test.trajectoryBuilder(startPose, startPose.heading)
//                            .splineTo(avoidStack, 0.0)
//                            .splineTo(dropFirstWobble[4]!!.vec(), dropFirstWobble[4]!!.heading)
//                            .build(),
//                    // go get second wobble
//                    test.trajectoryBuilder(dropFirstWobble[4]!!, dropFirstWobble[4]!!.heading + 180.0.toRadians)
//                            .splineTo(secondWobbleApproach.vec(), secondWobbleApproachAngle)
//                            .build()
//            )
    )
    fun getTrajectories(a: Int): List<AutoPathElement>{
        return trajectorySets[a]!!
    }

}