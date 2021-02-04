package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Bot;

@Autonomous(name = "Main Autonomous", group = "Competition")
public class MainAutonomous extends LinearOpMode {

  private Bot bot;

  RingStackDetector.RingStackResult rings;
  double ringConfidence;
  RingStackDetector pipeline;


//  static ConditionalPoseSet[] poseSets = new ConditionalPoseSet[5];
//  ConditionalPoseSet poseSet;


//  static {
//    /*
//    0 rings bot(intersects launch line)
//    1 ring mid
//    4 rings corner
//          May have to increase Y because of wobble goal arm
//     */
//    poseSets[0] = new ConditionalPoseSet(new Pose2d[]{new Pose2d(-2, -48, -90),
//        new Pose2d(-32, -59, 90),
//        new Pose2d(6, -48, -90)});
//    poseSets[1] = new ConditionalPoseSet(new Pose2d[]{new Pose2d(22, -24, -90.00001),
//        new Pose2d(-32, -59, 90),
//        new Pose2d(30, -24, -90)});
//    poseSets[4] = new ConditionalPoseSet(new Pose2d[]{//new Pose2d(-24, -20, 0),Plows through 4ring
//        new Pose2d(46, -48, -89.99999),
//        new Pose2d(-32, -59, 90),
//        new Pose2d(54, -48, -90)});
//  }

  @Override
  public void runOpMode() throws InterruptedException {
//    AutoPaths paths = new AutoPaths();

    // TODO next: keep running the pipeline and updating the verdict in the INIT loop
    bot = Bot.getInstance(this);

    while (!isStarted()) {
      if (isStopRequested()) return;
      // keep getting results from the pipeline
    }

    rings = (pipeline.currentlyDetected().orElse(null)).first;
    ringConfidence = (pipeline.currentlyDetected().orElse(null)).second;
//    List<Trajectory> trajectories = paths.getTrajectories(rings.ringCount);

    // get the pose set for the current ring stack setup
//    for(Trajectory trajectory : trajectories){
//      bot.roadRunner.followTrajectory(trajectory);//i guess
//    }


    // shoot 3 rings
    bot.shooter.shootRings(this, 3, 0.8);

    // follow the pose set

  }




//  private static class ConditionalPoseSet {
//
//    Pose2d[] poses;
//    DisplacementMarker[] markers;
//
//    private static class DisplacementMarker {
//
//      double dist;
//      Runnable action;
//
//      public DisplacementMarker(double dist, Runnable action) {
//        this.dist = dist;
//        this.action = action;
//      }
//    }
//
//    Consumer<TrajectoryBuilder> applyPoses = (TrajectoryBuilder builder) -> {
//      for (Pose2d pose : poses) {
//        builder.splineToLinearHeading(pose, 0);
//      }
//    };
//
//    Consumer<TrajectoryBuilder> applyMarkers = (TrajectoryBuilder builder) -> {
//      for (DisplacementMarker marker : markers) {
//        if (marker.dist == -1) {
//          builder.addDisplacementMarker(marker.action::run);
//        } else {
//          builder.splineToLinearHeading(new Pose2d(), 0.0);
//        }
//      }
//    };
//
//    public ConditionalPoseSet(Pose2d[] poses) {
//      this.poses = poses;
//    }
//  }
}

/*michael's kotlin stuff
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(40.0, 30.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPose = Pose2d(-48.0 - 24 + 9, -34.0, 0.0)

    // (-3.36345, -0.0756263), (72-22.75, 0)
    private val dropFirstWobble = mapOf(
        0 to Pose2d(-0.0756263, -72.0 + 22.75 + 3.36345, (-90.0 + 39.05465).toRadians),
        1 to Pose2d(24 - 2.668, -24 - 11.416 - 0.6, (-90 + 85.87698).toRadians),
        4. to Pose2d(48.0 + 1.4313, -48 + 5.35248, (-90 + 31.4772).toRadians)
    )
    private val ramIntoStack = Vector2d(-24.0, -24.0 - 12.0)
    private val avoidStack = Vector2d(-24.0, -160)


    private val secondWobbleLocation = Vector2d(-48.0, -48 - 2.0)
    private val secondWobbleApproachAngle = 160.0.toRadians
    private val secondWobbleApproach = Pose2d(secondWobbleLocation.plus(Vector2d(18.8722, -0.622)), secondWobbleApproachAngle)
    private val trajectorySets: Map<Int, List<Trajectory>> = mapOf(
        0 to run {
            val interm = Vector2d(5.0, -24 - 6.0)
            val intermAngle = 0.0
            val secondWobble = Pose2d(secondWobbleLocation.plus(Vector2d(14.073, 12.58958)), (294.2298 - 90.0).toRadians)
            val placeSecondWobble = Pose2d(-8.8717, -48.0 - 5.344)
            listOf(
                // go to drop first wobble
                TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)
                    .splineTo(dropFirstWobble[0]!!.vec(), dropFirstWobble[0]!!.heading)
                    .build(),
                // turn around
                TrajectoryBuilder(
                    dropFirstWobble[0]!!,
                    dropFirstWobble[0]!!.heading + 180.0.toRadians,
                    combinedConstraints
                )
                    .splineTo(interm, intermAngle)
                    .build(),
                // go to get second wobble
                TrajectoryBuilder(Pose2d(interm, intermAngle + Math.PI), intermAngle + Math.PI, combinedConstraints)
                    .splineTo(secondWobble.vec(), secondWobble.heading)
                    .build()
            )
        },
        1 to listOf(
            TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)
                .splineTo(ramIntoStack, 0.0)
                .splineTo(dropFirstWobble[1]!!.vec(), dropFirstWobble[1]!!.heading)
                .build(),
            TrajectoryBuilder(dropFirstWobble[1]!!, dropFirstWobble[1]!!.heading + 180.0.toRadians, combinedConstraints)
                .splineTo(secondWobbleApproach.vec(), secondWobbleApproachAngle)
                .build()),
        4 to listOf(
            TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)
                .splineTo(avoidStack, 0.0)
                .splineTo(dropFirstWobble[4]!!.vec(), dropFirstWobble[4]!!.heading)
                .build(),
            TrajectoryBuilder(dropFirstWobble[4]!!, dropFirstWobble[4]!!.heading + 180.0.toRadians, combinedConstraints)
                .splineTo(secondWobbleApproach.vec(), secondWobbleApproachAngle)
                .build())
    )

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

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))

val Pose2d.unit get() = this.div(this.vec().norm())
 */
