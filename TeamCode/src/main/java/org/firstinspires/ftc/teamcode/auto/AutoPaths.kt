package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import org.firstinspires.ftc.teamcode.hardware.Bot
import java.lang.Math.toRadians

class AutoPaths {
    private val startPose = Pose2d(-48.0 - 24 + 9, -34.0, 0.0)
    val bot = Bot.getInstance()
    val test = bot.roadRunner

    val Double.toRadians get() = (Math.toRadians(this))
    // (-3.36345, -0.0756263), (72-22.75, 0)
    private val ramIntoStack = Vector2d(-24.0, -24.0 - 12.0)
    private val avoidStack = Vector2d(-24.0, -160.0)
    private val dropFirstWobble = mapOf(
            0 to Pose2d(-0.0756263, -72.0 + 22.75 + 3.36345, (-90.0 + 39.05465).toRadians),
            1 to Pose2d(24 - 2.668, -24 - 11.416 - 0.6, (-90 + 85.87698).toRadians),
            4 to Pose2d(48.0 + 1.4313, -48 + 5.35248, (-90 + 31.4772).toRadians)
    )
    //private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth) //replace w/ constraints from DriveConstants

    private val interm = mapOf(
            0 to Pose2d(5.0, -24 - 6.0, 0.0),
            1 to Pose2d(),
            4 to Pose2d()
    )

    //getting second wobble goal for 1 and 4
    private val secondWobbleLocation = Vector2d(-48.0, -48 - 2.0)
    private val secondWobbleApproachAngle = 160.0.toRadians
    private val secondWobbleApproach = Pose2d(secondWobbleLocation.plus(Vector2d(18.8722, -0.622)), secondWobbleApproachAngle)


    private val trajectorySets: Map<Int, List<Trajectory>> = mapOf(
            0 to run {
                //gettting second wobble goal for 0
                val secondWobble = Pose2d(secondWobbleLocation.plus(Vector2d(14.073, 12.58958)), (294.2298 - 90.0).toRadians)
                //placing second wobble goal
                val placeSecondWobble = Pose2d(-8.8717, -48.0 - 5.344)
                listOf(
                        // go to drop first wobble
                        test.trajectoryBuilder(startPose, startPose.heading)
                                .splineTo(dropFirstWobble[0]!!.vec(), dropFirstWobble[0]!!.heading)
                                .build(),
                        // intermediate point turn
                        test.trajectoryBuilder(dropFirstWobble[0]!!, dropFirstWobble[0]!!.heading + 180.0.toRadians)
                                .splineTo(interm[0]!!.vec(), interm[0]!!.heading)
                                .build(),
                        // go to get second wobble
                        test.trajectoryBuilder(Pose2d(interm[0]!!.vec(), interm[0]!!.heading + Math.PI), interm[0]!!.heading + Math.PI)
                                .splineTo(secondWobble.vec(), secondWobble.heading)
                                .build(),
                        //go to deposit second wobble
                        test.trajectoryBuilder(secondWobbleApproach)
                                .splineTo(placeSecondWobble.vec(), placeSecondWobble.heading)
                                .build()
                )
            },
            1 to run {

                val pose2 = Pose2d(20.0, -35.0, toRadians(175.0))
                val pose3 = Pose2d(secondWobbleLocation.x, secondWobbleLocation.y, secondWobbleApproachAngle + Math.PI)

                listOf(
                        //ONE RING TRAJECTORY
                        test.trajectoryBuilder(startPose, startPose.heading)
                                .splineTo(ramIntoStack, 0.0)//optional?
                                .splineTo(dropFirstWobble[1]!!.vec(), dropFirstWobble[1]!!.heading)
                                .build(),

                        test.trajectoryBuilder(pose2, pose2.heading)
                                .splineTo(secondWobbleLocation, secondWobbleApproachAngle)
                                .build(),

                        test.trajectoryBuilder(pose3, pose3.heading)
                                .splineTo(Vector2d(16.0, -50.0), (12.0).toRadians)
                                .build()
                )
            },
            4 to run {
                //placing second wobble goal
                val placeSecondWobble = Pose2d(-8.8717, -48.0 - 5.344)
                listOf(
                        // go to drop first wobble
                        test.trajectoryBuilder(startPose, startPose.heading)
                                .splineTo(dropFirstWobble[4]!!.vec(), dropFirstWobble[4]!!.heading)
                                .build(),
                        // intermediate point turn
                        test.trajectoryBuilder(dropFirstWobble[4]!!, dropFirstWobble[4]!!.heading + 180.0.toRadians)
                                .splineTo(interm[4]!!.vec(), interm[4]!!.heading)
                                .build(),
                        // go to get second wobble
                        test.trajectoryBuilder(Pose2d(interm[4]!!.vec(), interm[4]!!.heading + Math.PI), interm[4]!!.heading + Math.PI)
                                .splineTo(secondWobbleApproach.vec(), secondWobbleApproach.heading)
                                .build(),
                        //go to deposit second wobble
                        test.trajectoryBuilder(secondWobbleApproach)
                                .splineTo(placeSecondWobble.vec(), placeSecondWobble.heading)
                                .build()
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
    fun getTrajectories(a: Int): List<Trajectory>{
        return trajectorySets[a]!!;
    }
    fun createTrajectory(): List<Trajectory> {

//        val steps1 = dropFirstWobble.map { (numRings, target) ->
//            val builder = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)
//            stackBehavior[numRings]?.let {
//                builder.splineTo(it, 0.0)
//            }
//            builder.splineTo(target.vec(), target.heading).build()
//        }
//
//        val steps2 = dropFirstWobble.map { (_, start) ->
//            val circularRadius = 8.0
//            val builder = TrajectoryBuilder(start, start.headingVec().rotated(180.0.toRadians).angle(), combinedConstraints)
//            val next = start.vec().minus(start.headingVec() * circularRadius).plus(start.headingVec().rotated(90.0.toRadians) * circularRadius)
//            builder.splineTo(next, next.minus(secondWobbleApproach.vec()).angle()).build()
//        }
//
//        return steps2

        return trajectorySets[0]!!
    }
}