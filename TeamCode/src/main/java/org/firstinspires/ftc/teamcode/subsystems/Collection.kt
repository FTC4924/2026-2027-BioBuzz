package org.firstinspires.ftc.teamcode.subsystems

import android.R
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Collection(hardwareMap: HardwareMap) {

    enum class CollectionState(val power: Double) {
        Forward(1.0),
        Backward(-0.75),
        Stopped(0.0)
    }

    var collectionState = CollectionState.Stopped   //keeps track of current state assumes that we start stopped

    public val collectMotor = hardwareMap.get(DcMotorSimple::class.java, "collection")
    private val collectServoLeft = hardwareMap.get(CRServo::class.java, "collectLeft")
    private val collectServoRight = hardwareMap.get(CRServo::class.java, "collectRight")

    var collectionPower = 0.0  //When program/class is initialized, assume start at 0

    init {
        //collectMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    inner class SetState(private val state: CollectionState, private val justServos: Boolean = false) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {     //! means not
                collectionPower = state.power

                if (!justServos) collectMotor.power = collectionPower

                collectServoLeft.power = collectionPower
                collectServoRight.power = -collectionPower
                collectionState = state
                initialized = true
            }
            return false
        }
    }

    fun collectIn(servoOnly: Boolean = false): Action = SetState(CollectionState.Forward, servoOnly)
    fun spitOut(servoOnly: Boolean = false): Action = SetState (CollectionState.Backward, servoOnly)
    fun stop(): Action = SetState(CollectionState.Stopped, false)

}