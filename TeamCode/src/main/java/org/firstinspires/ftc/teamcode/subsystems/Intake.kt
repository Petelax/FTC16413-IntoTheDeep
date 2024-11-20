package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.utils.Cache
import kotlin.math.abs
import kotlin.math.max

class Intake(hardwareMap: HardwareMap): SubsystemBase() {
    private var left: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_LEFT)
    private var right: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_RIGHT)
    private var color: RevColorSensorV3 = hardwareMap.get(RevColorSensorV3::class.java, "color")

    private var red = 0.0
    private var green = 0.0
    private var blue = 0.0
    private var max = 0.0
    private var colors = NormalizedRGBA()

    private var distance = 0.0

    private var lastSpeed = -100.0

    private var count = 0
    private var countsPerPeriodic = 1

    private var colourEnable = false

    init {
        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.direction = DcMotorSimple.Direction.REVERSE
        left.power = 0.0
        right.power = 0.0
        color.initialize()
    }

    override fun periodic() {
        //if (abs(lastSpeed) >= 0.01 || abs(lastSpeed) > 10.0) {
            //if (count % countsPerPeriodic == 0) {
                count = 0
                colors = color.normalizedColors
                /*
                red = color.red().toDouble()
                green = color.green().toDouble()
                blue = color.blue().toDouble()
                 */
                red = colors.red.toDouble()
                //green = colors.green.toDouble()
                blue = colors.blue.toDouble()

                max = max(red, max(green, blue))

                red /= max
                blue /= max
                //green /= max

                distance = color.getDistance(DistanceUnit.MM)
            //}
            //++count
        //}

    }

    fun getRed(): Double {
        return red
    }

    fun getBlue(): Double {
        return blue
    }

    fun getGreen(): Double {
        return green
    }

    /**
     * mm
     */
    fun getDistance(): Double {
        return distance
    }

    fun getSpeed(): Double {
        return lastSpeed
    }

    fun getGamePiece(): Sample {
        if (distance >= 20.0) {
            return Sample.NONE
        }
        if (getRed() > 0.95 && getBlue() < 0.5) {
            return Sample.RED
        }
        if (getRed() < 0.5 && getBlue() > 0.95) {
            return Sample.BLUE
        }

        return Sample.YELLOW

    }

    fun setSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        //if ((lastSpeed != corrected)) {
        if (Cache.shouldUpdate(lastSpeed, corrected, 0.005)) {
        //if (lastSpeed.isNaN() || (abs(lastSpeed-corrected) >= 0.005) || (lastSpeed != 0.0 && corrected == 0.0) || (corrected != 0.0 && lastSpeed == 0.0)) {
            left.power = corrected
            right.power = corrected
            lastSpeed = corrected
        }
        //}
    }

    fun stop() {
        left.power = 0.0
        right.power = 0.0
    }

    enum class Sample {
        NONE,
        YELLOW,
        RED,
        BLUE
    }

}
