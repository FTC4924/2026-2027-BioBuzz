package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
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
import org.firstinspires.ftc.teamcode.subsystems.Pin
import org.firstinspires.ftc.teamcode.subsystems.Transfer
import org.firstinspires.ftc.teamcode.subsystems.Catapult
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import kotlin.math.PI

@TeleOp
class BiobuzzTeleop : OpMode() {
    private lateinit var drive: MecanumDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad

    private lateinit var collection: Collection
    private lateinit var pin: Pin
    private lateinit var transfer: Transfer
    private lateinit var catapult: Catapult

    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0
    private lateinit var lastHeading: Rotation2d


    override fun init() {
        drive = MecanumDrive(hardwareMap,LocationShare.robotLocation)
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        collection = Collection(hardwareMap)
        pin = Pin(hardwareMap)
        transfer = Transfer(hardwareMap)
        catapult = Catapult(hardwareMap)
        lastHeading = drive.localizer.pose.heading
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

        telemetry.addData("seconds/loop", deltaTime)

        telemetry.addData("x", robotPose.position.x)
        telemetry.addData("y", robotPose.position.y)
        telemetry.addData("heading (deg)", Math.toDegrees(robotPose.heading.toDouble()))

        //update drive Pose
        drive.updatePoseEstimate()

        val driverHeading = 0       // all rotations/heading things in radians
        val rawHeading = robotPose.heading
        val heading: Rotation2d =
            Rotation2d.fromDouble(rawHeading.toDouble() - headingOffset - driverHeading)
        val slowSpeed = 0.5 //Normally go about 80% of our fastest speed

        /* driver 1 */
        packet.field().setRotation(Math.PI / 2)
        packet.fieldOverlay().setRotation(-Math.PI / 2)
        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), robotPose)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        //telemetry.addData("Ramp position", ramp.ramp.currentPosition)

        dash.sendTelemetryPacket(packet)

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

        //  collection control, hold a button to turn on motor and servo, hold bumper to turn off motor
        if (g2.a.isHeld()) {
            runningActions.add(collection.collectIn(g2.leftBumper.isHeld()))
        }
        if (g2.y.isHeld()) {
            runningActions.add(collection.spitOut(g2.leftBumper.isHeld()))
        }
        if (g2.a.isInactive() and g2.y.isInactive()) {
            runningActions.add(collection.stop())
        }
        if (g2.leftBumper.justPressed() and (collection.collectMotor.power != 0.0)) {
            runningActions.add(collection.stop())
        }

        /*if (g1.dpadRight.justPressed()) {
            runningActions.add(pin.Locked())
        }
        if (g1.dpadLeft.justPressed()) {
            runningActions.add(pin.Shoot())
        }*/

        if (g2.b.justPressed()) { // pin will shoot, catapult will unwind, then pin will re-lock
            runningActions.add(             //The function on g2.b is untested as of 11/17
                SequentialAction(
                    pin.Shoot(),
                    SleepAction(1.0),
                    catapult.Release(),
                    SleepAction(2.0),
                    pin.Locked(),

                    )
            )
        }

        if (g2.rightBumper.isHeld()) {
            runningActions.add(transfer.loadBucket())
        }
        if (g2.rightBumper.justReleased()) {
            runningActions.add(transfer.readyToCollect())
        }

        if (g2.dpadDown.justPressed()) {
            runningActions.add(catapult.Light())
        }
        if (g2.dpadRight.justPressed()) {
            runningActions.add(catapult.Medium())
        }
        if (g2.dpadUp.justPressed()) {
            runningActions.add(catapult.Heavy())
        }
        if (g2.dpadLeft.justPressed()) {
            runningActions.add(catapult.SuperUltraPower())
        }
        /*if (g2.b.justPressed()) {
            runningActions.add(catapult.Release())
        } */
        if (g2.leftStickY.isActive()) {
            runningActions.add(catapult.manual(g2.leftStickY.component * deltaTime))
            // The "deltaTime" is needed above to stop the motor after you let go of triggers
        }

        if (g2.leftTrigger.isActive() or g2.rightTrigger.isActive()){
            runningActions.add(
                transfer.manual(g2.leftTrigger.component, g2.rightTrigger.component)
            )
        }

    }

}