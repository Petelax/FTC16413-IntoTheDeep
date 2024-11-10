package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import kotlin.math.abs

class Elevator(hardwareMap: HardwareMap): SubsystemBase() {
    private var motorLeft: Motor
    private var motorRight: Motor

    private var elevator: MotorGroup
    private var currentPosition: Double = 0.0
    private var lastSpeed = 0.0

    private val positions = VerticalConstants.ElevatorPositions
    private val coefficients = VerticalConstants.ElevatorCoefficients
    private val constants = VerticalConstants.ElevatorConstants

    init {
        val ff = coefficients
        motorLeft = Motor(hardwareMap, DeviceIDs.ELEVATOR_LEFT, Motor.GoBILDA.RPM_435)
        motorRight = Motor(hardwareMap, DeviceIDs.ELEVATOR_RIGHT, Motor.GoBILDA.RPM_435)

        motorLeft.stopAndResetEncoder()
        motorRight.stopAndResetEncoder()

        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        motorRight.inverted = true

        elevator = MotorGroup(motorLeft, motorRight)

    }

    fun setRawSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            elevator.set(corrected)
            lastSpeed = corrected
        }
    }

    fun setSpeed(speed: Double) {
        currentPosition = getPosition()

        if ((currentPosition < positions.LOWER_LIMIT && speed <= 0.0) || (currentPosition > positions.UPPER_LIMIT && speed > 0.0)) {
            setRawSpeed(0.0)
        } else {
            setRawSpeed(speed + coefficients.KG)
        }

    }

    fun getRawPosition(): Int {
        return motorLeft.currentPosition
    }

    fun getPosition(): Double {
        return getRawPosition() * constants.TICKS_TO_INCHES
    }

}
