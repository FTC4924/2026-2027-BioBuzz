package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.HardwareMap

class Pin(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class PinPosition(val position: Double) {
        Locked(1.0),
        Shoot(0.0),

    }


    var pinPosition = PinPosition.Locked        // keeps track of our state. Start in locked!

    // initialize our servo, pin, and map it from the hardware file
    private val pin = hardwareMap.get(Servo::class.java, "pin")

    //Whenever we are moving the pin, we use this class to set the state
    inner class SetState(private val state: PinPosition) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                pinPosition = state // keep track of what state we are in
                pin.position = state.position
                initialized = true
            }
            return false
        }
    }

    fun Locked(): Action = SetState(PinPosition.Locked)
    fun Shoot(): Action = SetState (PinPosition.Shoot)

}