package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.Cache

class HorizontalExtension(hardwareMap: HardwareMap): SubsystemBase() {
    private var motor: DcMotorEx

    //private var limit: TouchSensor

    private var currentPosition: Double = 0.0
    private var positionOffset = 0.0
    private var lastSpeed = 0.0
    private var atBottom = true
    private var lastAtBottom = false
    private var currentLeft = 0.0
    private var currentRight = 0.0
    private var speed = 0.0

    private val positions = HorizontalConstants.HorizontalExtensionPositions
    private val coefficients = HorizontalConstants.HorizontalExtensionCoefficients
    private val constants = HorizontalConstants.HorizontalExtensionConstants

    init {
        motor = hardwareMap.get(DcMotorEx::class.java, DeviceIDs.HORIZONTAL_EXTENSION)
        //limit = hardwareMap.touchSensor.get(DeviceIDs.HORIZONTAL_LIMIT)

        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    }

    override fun periodic() {
        currentPosition = (motor.currentPosition * constants.TICKS_TO_INCHES) - positionOffset
        //speed = motor.velocity * constants.TICKS_TO_INCHES
        //currentLeft = motor.getCurrent(CurrentUnit.AMPS)
        /*
        atBottom = limit.isPressed
        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition
        }

        lastAtBottom = atBottom
         */
    }

    fun setRawSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            //elevator.set(corrected)
            motor.power = corrected
            lastSpeed = corrected
        }
    }

    fun setSpeed(speed: Double) {
        currentPosition = getPosition()

        if ((currentPosition < positions.LOWER_LIMIT && speed <= 0.0) || (currentPosition > positions.UPPER_LIMIT && speed > 0.0)) {
            setRawSpeed(0.0)
        } else {
            setRawSpeed(speed)
        }

    }

    fun getPosition(): Double {
        return currentPosition
    }

    fun atBottom(): Boolean {
        return atBottom
    }

    fun getOffset(): Double {
        return positionOffset
    }

    fun getSpeed(): Double {
        return speed
    }

    fun getCurrent(): Double {
        return currentLeft
    }

}
