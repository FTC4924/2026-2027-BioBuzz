package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.max
import kotlin.math.min
class Transfer(hardwareMap: HardwareMap) {

    /**
     * @param positionL the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class TransferPosition(val positionL: Double,val positionR: Double) {
        Up(0.10, 0.30),
        Down(0.30, 0.10),
        Manual(-1.0,-1.0)
    }


    var transferPosition = TransferPosition.Down        // keeps track of our state. Start in down!

    // initialize our servo, transfer, and map it from the hardware file
    private val transferL = hardwareMap.get(Servo::class.java, "transferL")
    private val transferR = hardwareMap.get(Servo::class.java, "transferR")

    //Whenever we are moving the pin, we use this class to set the state
    inner class SetState(private val state: TransferPosition) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                transferPosition = state // keep track of what state we are in
                transferL.position = state.positionL
                transferR.position = state.positionR
                initialized = true
            }
            return false
        }
    }

    inner class Manual(private val leftTrig: Double, private val rightTrig: Double) : Action {
        private var initialized = false
        //The below should be changed to use maximums and minimums but should work as long as servos are mirrored
        private val delta = TransferPosition.Up.positionR - TransferPosition.Down.positionR

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                transferPosition = TransferPosition.Manual // keep track of what state we are in
                transferL.position = (leftTrig * delta) + TransferPosition.Down.positionR
                transferR.position = -(rightTrig * delta) + TransferPosition.Down.positionL
                initialized = true
            }
            return false
        }

    }

    fun loadBucket(): Action = SetState(TransferPosition.Up)
    fun readyToCollect(): Action = SetState (TransferPosition.Down)
    fun manual(leftTrig: Double, rightTrig: Double) = Manual(leftTrig, rightTrig)

}