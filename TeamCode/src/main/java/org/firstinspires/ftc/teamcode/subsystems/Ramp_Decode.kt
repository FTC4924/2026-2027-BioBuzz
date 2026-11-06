package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit


class Ramp_Decode(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class RampState(val position: Int) {
        Index(445), // was 450 pre 1/19
        Shoot(470), // was 360 pre 1/14
        Collect(1070), // was 1060 pre 1/19
        Partner (0), //
        Manual(-1)
    }

    var rampState = RampState.Collect

    val ramp = hardwareMap.get(DcMotorEx::class.java, "ramp")

    private val power = 0.75

    var rampOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0    //When program/class is initialized, assume start at 0

    init {
        ramp.setCurrentAlert(4.5, CurrentUnit.AMPS)

        ramp.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ramp.targetPosition = 0
        ramp.mode = DcMotor.RunMode.RUN_TO_POSITION
        ramp.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: RampState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                ramp.targetPosition = targetPosition.toInt() + rampOffset
                rampState = state
                initialized = true
            }
            //scoringArm.currentPosition
            packet.put("Target Position", ramp.targetPosition)
            packet.put("Current Position", ramp.currentPosition)

            return ramp.isBusy
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */
    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 600.0 //tics/second

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            rampState = RampState.Manual
            targetPosition += input * maxSpeed
            ramp.targetPosition = targetPosition.toInt() + rampOffset
            packet.put("Target Position", ramp.targetPosition)
            return false
        }
    }

    /**
     * zeros the ramp at the lowest position using current detection
     */
    inner class Zero() : Action {
        // how far the target position needs to be in the opposite direction to stop stalling
        val StallOffset = 70

        private var initialized = false

        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                initialized = true
                ramp.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                ramp.power = -0.5
            }
            val current = ramp.getCurrent(CurrentUnit.AMPS)
            packet.put("Current", current)
            if (current > 2.8) {
                ramp.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                rampOffset = StallOffset
                ramp.targetPosition = StallOffset
                ramp.mode = DcMotor.RunMode.RUN_TO_POSITION
                ramp.power = power
                return false
            }
            return true
        }
    }

    fun shoot(): Action = SetState(RampState.Shoot)
    fun collect(): Action = SetState(RampState.Collect)
    fun index(): Action = SetState(RampState.Index)
    fun homeRamp(): Action = Zero()
    fun toZero(): Action = SetState(RampState.Partner)
    fun manual(input: Double): Action = Manual(input)

    fun getPosition(): Int {
        return ramp.currentPosition
    }
}

