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
import org.firstinspires.ftc.teamcode.subsystems.Catapult
import org.firstinspires.ftc.teamcode.subsystems.Collection
import org.firstinspires.ftc.teamcode.subsystems.Collection.CollectionState

@TeleOp
class CatapultTesting : OpMode() {
    private lateinit var drive: MecanumDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad

    private lateinit var catapult: Catapult

    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0

    override fun init() {
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        catapult = Catapult(hardwareMap)
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


        if (g1.a.justPressed()) {
            runningActions.add(catapult.Light())//5 Rotations
        }
        if (g1.x.justPressed()) {
            runningActions.add(catapult.Medium())//6 Rotations
        }
        if (g1.y.justPressed()) {
            runningActions.add(catapult.Heavy())//7 Rotations
        }
        if (g1.b.justPressed()){
            runningActions.add(catapult.SuperUltraPower())//9 Rotations
        }
        if (g1.dpadDown.justPressed()) {
            runningActions.add(catapult.Release())
        }
        if (g1.leftTrigger.isActive() or g1.rightTrigger.isActive()) {
            val trigger = gamepad1.left_trigger - gamepad1.right_trigger
            runningActions.add(catapult.Manual(trigger.toDouble()))
        }
    }

}