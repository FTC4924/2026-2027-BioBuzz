package org.firstinspires.ftc.teamcode.autonomous
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.LocationShare
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive
import org.firstinspires.ftc.teamcode.subsystems.Collection
import org.firstinspires.ftc.teamcode.subsystems.Ramp
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.teleops.PandaTelemetryPacket


@Autonomous
class ShootBlueGoal : OpMode() {

    val beginPose = Pose2d (-50.0,-51.0,Math.toRadians(60.0))
    val shootPose = Pose2d (-48.0,-15.0,Math.toRadians(-23.0))
    lateinit var shooter: Shooter
    lateinit var drive: IHDrive
    lateinit var ramp: Ramp
    lateinit var collection: Collection
    var finished = false
    lateinit var autoAction: Action

    override fun init() {
        shooter = Shooter(hardwareMap)
        drive = IHDrive(hardwareMap, beginPose)
        ramp = Ramp(hardwareMap)
        collection = Collection(hardwareMap)

        autoAction = SequentialAction(
            drive.actionBuilder(beginPose)
                .setTangent(3 * Math.PI/4)
                .splineToSplineHeading(
                   shootPose,
                    Math.PI)
                .build(),
            //shooter.adjustPower(-.03),
            ramp.collect(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(1.0)
                .build(),
            shooter.shoot(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(3.0)
                .build(),
            shooter.feed(),
            collection.collectIn(),
            ramp.shoot(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(2.0)
                .build(),
            shooter.adjustPower(-0.05),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(1.0)
                .build(),
            ramp.index(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(1.5)
                .build(),
            ramp.shoot(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(1.0)
                .build(),
            ramp.index(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(1.5)
                .build(),
            ramp.shoot(),
            drive.actionBuilder(drive.localizer.pose)
                .waitSeconds(5.0)
                .build(),
            shooter.stop(),
            shooter.stopFeeding(),
            collection.stop(),
            ramp.toZero(),

            )
    }

    override fun loop() {
        val packet = PandaTelemetryPacket(telemetry)
        if (!finished) {
            if (!autoAction.run(packet)) finished = true
        }
        drive.updatePoseEstimate()

        telemetry.addData("x", drive.localizer.pose.position.x)
        telemetry.addData("y", drive.localizer.pose.position.y)
        telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.pose.heading.toDouble()))
    }
    override fun stop() {
        LocationShare.robotLocation = drive.localizer.pose
    }

}
