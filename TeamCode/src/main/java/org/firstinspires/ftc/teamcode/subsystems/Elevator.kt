package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants

class Elevator(hardwareMap: HardwareMap): SubsystemBase() {
    private var motorLeft: Motor
    private var motorRight: Motor

    private var elevator: MotorGroup
    private var currentPosition: Double = 0.0

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
        elevator.set(speed)
    }

    fun setSpeed(speed: Double) {
        currentPosition = getPosition()

        if ((currentPosition < positions.BOTTOM && speed < 0.0) || (currentPosition > positions.TOP && speed > 0.0)) {
            elevator.set(0.0)
        } else {
            elevator.set(speed + coefficients.KG)
        }

    }

    fun getRawPosition(): Int {
        return motorLeft.currentPosition
    }

    fun getPosition(): Double {
        return getRawPosition() * constants.TICKS_TO_INCHES
    }

}
