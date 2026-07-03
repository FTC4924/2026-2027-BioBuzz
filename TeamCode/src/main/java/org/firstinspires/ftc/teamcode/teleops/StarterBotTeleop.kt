package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LocationShare
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Collection
import org.firstinspires.ftc.teamcode.subsystems.Collection.CollectionState
import kotlin.math.PI

@TeleOp
class StarterBotTeleop : OpMode() {
    private lateinit var drive: MecanumDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad

    private lateinit var collection: Collection

    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0


    override fun init() {
        drive = MecanumDrive(hardwareMap,LocationShare.robotLocation)
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        collection = Collection(hardwareMap)
    }

    override fun start() {
        lastTime = runtime
    }

    fun getDeltaTime(): Double {
        val newTime = runtime
        val deltaTime: Double = newTime - lastTime
        lastTime = newTime
        return deltaTime
    }

    override fun loop() {
        val robotPose = drive.localizer.pose

        //update delta time
        val deltaTime = getDeltaTime()
        //telemetry.addData("seconds/loop", deltaTime)

        //update gamepad values
        g1.update()
        g2.update()

        //run and update actions
        val packet = PandaTelemetryPacket(telemetry)

        val newActions: MutableList<Action> = ArrayList()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            //telemetry.addLine(action.toString())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        telemetry.addData("x", robotPose.position.x)
        telemetry.addData("y", robotPose.position.y)
        telemetry.addData("heading (deg)", Math.toDegrees(robotPose.heading.toDouble()))

        packet.field().setRotation(Math.PI / 2)
        packet.fieldOverlay().setRotation(-Math.PI / 2)
        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), robotPose)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        //telemetry.addData("Ramp position", ramp.ramp.currentPosition)

        dash.sendTelemetryPacket(packet)

        //update drive Pose
        drive.updatePoseEstimate()

        /* driver 1 */
        val driverHeading = PI/2
        val rawHeading = drive.localizer.pose.heading
        val heading: Rotation2d = Rotation2d.fromDouble(rawHeading.toDouble() - headingOffset - driverHeading)
        val slowSpeed = 0.8 //Normally go about 80% of our fastest speed

        val leftStick1 = Vector2d(
            g1.leftStickY.component,
            -g1.leftStickX.component
        )


        if (g1.a.isHeld()) {  //When Slowing
            drive.setDrivePowers(
                PoseVelocity2d(
                    heading.inverse().times(leftStick1 * slowSpeed),
                    ((gamepad1.left_trigger - gamepad1.right_trigger) * 0.5 * slowSpeed)
                )
            )

        } else {
            drive.setDrivePowers(
                PoseVelocity2d(
                    heading.inverse().times(leftStick1),    //Coach Ethan added slow 1/19
                    ((gamepad1.left_trigger - gamepad1.right_trigger) * 0.5)
                )
            )
        }

        if (g1.b.justPressed()) headingOffset = rawHeading.toDouble() - driverHeading




        /* driver 2 */
        if (g2.x.justPressed()) {
            if (collection.collectionState == CollectionState.Stopped) runningActions.add(collection.collectIn())
            else if (collection.collectionState == CollectionState.Backward) runningActions.add(collection.collectIn())
            else runningActions.add(collection.stop())
        }
        if (g2.y.justPressed()) {
            if (collection.collectionState == CollectionState.Stopped) runningActions.add(collection.spitOut())
            else if (collection.collectionState == CollectionState.Forward) runningActions.add(collection.spitOut())
            else runningActions.add(collection.stop())
        }

    }

}