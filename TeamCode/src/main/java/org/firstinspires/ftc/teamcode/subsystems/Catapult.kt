package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit


class Catapult(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class SpoolState(val position: Double) {
        //Number is in Rotations. Not degrees or ticks
        Released(0.0),
        Light(3.5),
        Medium(4.0),//Previous tested amount for 1 pollen and 1 nectar
        Heavy(4.5),
        SuperUltraPower (5.0),
        Manual(-1.0)
    }

    var spoolState = SpoolState.Released       //start as released and keep track of current spool state.

    // creating motor object
    val spool = hardwareMap.get(DcMotorEx::class.java, "spool")

    private val power = 0.75

    var targetPosition = 0.0    //When program/class is initialized, assume start at 0

    init {
       // spool.setCurrentAlert(4.5, CurrentUnit.AMPS)
        spool.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE //Whenever the motor is not moving hold position.
        spool.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        spool.targetPosition = 0
        spool.mode = DcMotor.RunMode.RUN_TO_POSITION
        spool.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    val TPR: Int = 1425 //Ticks per rotation of the motor //PPR on goBilda

    inner class SetState(private val state: SpoolState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position * TPR
                spool.targetPosition = targetPosition.toInt()
                spoolState = state
                initialized = true
            }
            //scoringArm.currentPosition
            packet.put("Target Position", spool.targetPosition)
            packet.put("Current Position", spool.currentPosition)

            return spool.isBusy
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */
    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 1000.0 //tics/second

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            spoolState = SpoolState.Manual
            targetPosition += input * maxSpeed
            spool.targetPosition = targetPosition.toInt()
            packet.put("Target Position", spool.targetPosition)
            return false
        }
    }


    fun Medium(): Action = SetState(SpoolState.Medium)
    fun Heavy(): Action = SetState(SpoolState.Heavy)
    fun Light(): Action = SetState(SpoolState.Light)
    fun Release(): Action = SetState(SpoolState.Released)
    fun SuperUltraPower(): Action = SetState(SpoolState.SuperUltraPower)
    fun manual(input: Double): Action = Manual(input)

}

