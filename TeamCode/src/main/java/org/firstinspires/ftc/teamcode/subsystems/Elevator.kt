package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import kotlin.math.abs

class Elevator(hardwareMap: HardwareMap): SubsystemBase() {
    //private var motorLeft: Motor
    //private var motorRight: Motor

    private var motorLeft: DcMotorEx
    private var motorRight: DcMotorEx

    //private var elevator: MotorGroup
    private var currentPosition: Double = 0.0
    private var lastSpeed = 0.0

    private val positions = VerticalConstants.ElevatorPositions
    private val coefficients = VerticalConstants.ElevatorCoefficients
    private val constants = VerticalConstants.ElevatorConstants

    init {
        //motorLeft = Motor(hardwareMap, DeviceIDs.ELEVATOR_LEFT, Motor.GoBILDA.RPM_435)
        //motorRight = Motor(hardwareMap, DeviceIDs.ELEVATOR_RIGHT, Motor.GoBILDA.RPM_435)
        motorLeft = hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_LEFT)
        motorRight = hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_RIGHT)

        motorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        motorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorRight.direction = DcMotorSimple.Direction.REVERSE

        //motorLeft.stopAndResetEncoder()
        //motorRight.stopAndResetEncoder()

        //motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        //motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        //motorRight.inverted = true

        //elevator = MotorGroup(motorLeft, motorRight)

    }

    override fun periodic() {
        currentPosition = motorLeft.currentPosition * constants.TICKS_TO_INCHES
    }

    fun setRawSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            //elevator.set(corrected)
            motorLeft.power = corrected
            motorRight.power = corrected
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

    fun getPosition(): Double {
        return currentPosition
    }

}