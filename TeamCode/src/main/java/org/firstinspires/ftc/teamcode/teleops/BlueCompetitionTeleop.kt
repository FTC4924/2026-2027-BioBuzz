package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LocationShare
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive
import org.firstinspires.ftc.teamcode.subsystems.Collection
import org.firstinspires.ftc.teamcode.subsystems.Collection.CollectionState
import org.firstinspires.ftc.teamcode.subsystems.Ramp
import org.firstinspires.ftc.teamcode.subsystems.Shooter

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

@TeleOp
class BlueCompetitionTeleop : OpMode() {
    private lateinit var drive: IHDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad

    private lateinit var shooter: Shooter
    private lateinit var collection: Collection
    private lateinit var ramp: Ramp


    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0
    private val GoalPosition = Vector2d(-72.0, -72.0)

    private val shootUpInc: Double = 0.1
    private val shootDownInc: Double = 0.1


    val rightTriggerMax = PandaGamepad.AnalogComponent(0.95)
    val leftTriggerMax = PandaGamepad.AnalogComponent(0.95)

    override fun init() {
        drive = IHDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        shooter = Shooter(hardwareMap)
        ramp = Ramp(hardwareMap)
        collection = Collection(hardwareMap)
        drive.localizer.pose = LocationShare.robotLocation
    }

    override fun start() {
        lastTime = runtime
        //runningActions.add(ramp.homeRamp())
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

        rightTriggerMax.update(gamepad2.right_trigger.toDouble())
        leftTriggerMax.update(gamepad2.left_trigger.toDouble())

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

        telemetry.addData("ramp position", ramp.getPosition())
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
        val driverHeading = -PI/2
        val rawHeading = drive.localizer.pose.heading
        val heading: Rotation2d = Rotation2d.fromDouble(rawHeading.toDouble() - headingOffset - driverHeading)
        val slowSpeed = 0.8 //Normally go about 80% of our fastest speed

        val input = Vector2d(
            g1.leftStickY.component.toDouble(),
            -g1.leftStickX.component.toDouble()
        )

        if(g1.x.isHeld()) {
            // The position and angle of the shooter relative to the robot
            val shooterRelativeToRobotPose = Pose2d(
                Vector2d(7.0,0.0), -PI/2
            )

            // The angle from the center of the robot to the shooter relative to the robot
            val robotToShooterAngleRelative = 0.0

            // The distance between the center of the robot and the shooter
            val robotToShooterRadius = sqrt(
                shooterRelativeToRobotPose.position.x.pow(2.0) + shooterRelativeToRobotPose.position.y.pow(2.0)
            )

            // The angle from the center of the robot to the shooter relative to the field
            val robotToShooterAngleGlobal = robotToShooterAngleRelative + robotPose.heading.toDouble()

            // The position and angle of the shooter relative to the field
            val shooterPose = Pose2d(
                Vector2d(
                    robotPose.position.x + robotToShooterRadius * cos(robotToShooterAngleGlobal),
                    robotPose.position.y + robotToShooterRadius * sin(robotToShooterAngleGlobal)
                ),
                shooterRelativeToRobotPose.heading * robotPose.heading
            )

            // The position of the goal relative to the shooter
            val shooterRelativeToGoalPosition = Vector2d(
                GoalPosition.x - shooterPose.position.x,
                GoalPosition.y - shooterPose.position.y
            )

            // The angle from the shooter to the goal relative to the field
            val shooterToGoalAngle = atan2(shooterRelativeToGoalPosition.y, shooterRelativeToGoalPosition.x)

            //val robotToGoalPosition = Vector2d(
            //    GoalPosition.x - drive.pose.position.x,
            //    GoalPosition.y - drive.pose.position.y
            //)
            //val robotToGoalAngle = atan2(robotToGoalPosition.y,robotToGoalPosition.x) + PI/2

            // proportional constant for turning, increase to turn more sharply, decrease to turn slower
            val kP = 1.0
            val kFeedforward = 0.05

            val error = Rotation2d.exp(shooterToGoalAngle) - shooterPose.heading
            val errorScaled = error * kP + kFeedforward * (error/abs(error))

            drive.setDrivePowers(
                PoseVelocity2d(
                    heading.inverse().times(input * slowSpeed),
                    errorScaled
                )
            )

            telemetry.addData("error", errorScaled)
            telemetry.addData("targetAngle (deg)", Math.toDegrees(shooterToGoalAngle))

        } else {
            if (g1.a.isActive()) {  //When Boosting
                drive.setDrivePowers(
                    PoseVelocity2d(
                        heading.inverse().times(input * slowSpeed),
                        ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2 * slowSpeed).toDouble()
                    )
                )
            } else if (g1.y.isHeld()) {
                drive.setDrivePowers(
                    PoseVelocity2d(
                        heading.inverse().times(input),    //Coach Ethan added slow 1/19
                        ((gamepad1.left_trigger - gamepad1.right_trigger) * 2).toDouble()
                    )
                )

            } else {
                drive.setDrivePowers(
                    PoseVelocity2d(
                        heading.inverse().times(input),    //Coach Ethan added slow 1/19
                        ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2).toDouble()
                    )
                )
            }


            if (g1.b.justPressed()) headingOffset = rawHeading.toDouble() - driverHeading

        }


        /* driver 2 */

        if (g2.a.justPressed()) runningActions.add(shooter.shoot())
        else if (g2.a.justReleased()) runningActions.add(shooter.idle())

        if (g2.rightBumper.justPressed()) runningActions.add(shooter.feed())
        else if (g2.rightBumper.justReleased()) runningActions.add(shooter.stopFeeding())

        if (g2.dpadUp.justPressed()) runningActions.add(ramp.collect())

        if (g2.dpadRight.justPressed()) {
            runningActions.add(ramp.shoot())
        }

        if (g2.dpadLeft.justPressed()) {
            runningActions.add(ramp.index())
        } else if (g2.dpadLeft.justReleased()) {
            runningActions.add(ramp.shoot())
        }

        if (g2.dpadDown.justPressed()) runningActions.add(ramp.toZero())
        if (g2.leftBumper.justPressed()) runningActions.add(ramp.homeRamp())

        if (g2.leftStickY.isActive()) runningActions.add(ramp.manual(g2.leftStickY.component * deltaTime))

        if (g2.rightStickX.isActive()) shooter.belt.power = g2.rightStickX.component

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


        if (g2.rightTrigger.justActive()) runningActions.add(shooter.adjustPower(shootUpInc))
        else if (g2.rightTrigger.justInactive()) runningActions.add(shooter.adjustPower(-shootUpInc))

        if (rightTriggerMax.justActive()) runningActions.add(shooter.adjustPower(shootUpInc))
        else if (rightTriggerMax.justInactive()) runningActions.add(shooter.adjustPower(-shootUpInc))

        if (g2.leftTrigger.justActive()) runningActions.add(shooter.adjustPower(-shootDownInc))
        else if (g2.leftTrigger.justInactive()) runningActions.add(shooter.adjustPower(shootDownInc))

        if (leftTriggerMax.justActive()) runningActions.add(shooter.adjustPower(-shootDownInc))
        else if (leftTriggerMax.justInactive()) runningActions.add(shooter.adjustPower(shootDownInc))
    }

}