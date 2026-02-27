package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Shooter(hardwareMap: HardwareMap) {


    var powerAdjustment = 0.0

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ShooterState(val power: Double) {
        Shooting(0.72), //was 80 1/25
        Stopped(0.0),
        Idle(0.0)      //Need to test and update!!!
    }

    var shooterState = ShooterState.Stopped

    enum class BeltState(val power: Double) {
        Feeding(0.40), //was 80 1/25
        Stopped(0.0),
    }

    var beltState = BeltState.Stopped

    val shooter = hardwareMap.get(CRServo::class.java, "shooter")
    val belt = hardwareMap.get(CRServo::class.java, "belt")

    init {
        shooter.direction = DcMotorSimple.Direction.REVERSE
        shooter.power = ShooterState.Stopped.power
        belt.power = BeltState.Stopped.power
    }


    inner class SetShooterState(private val state: ShooterState) : Action {

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            shooterState = state
            shooter.power = shooterState.power + powerAdjustment
            return false
        }
    }


    inner class AdjustShooterPower(private val powerChange: Double)  : Action {

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            powerAdjustment += powerChange
            shooter.power = shooterState.power + powerAdjustment
            belt.power = shooterState.power + powerAdjustment
            return false
        }

    }

    inner class SetBeltState(private val state: BeltState) : Action {

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            beltState = state
            belt.power = beltState.power
            return false
        }
    }

    fun shoot(): Action = SetShooterState(ShooterState.Shooting)
    fun stop(): Action = SetShooterState(ShooterState.Stopped)
    fun idle(): Action = SetShooterState (ShooterState.Idle)
    fun adjustPower(powerChange: Double): Action = AdjustShooterPower(powerChange)

    fun feed(): Action = SetBeltState(BeltState.Feeding)
    fun stopFeeding(): Action = SetBeltState(BeltState.Stopped)
}