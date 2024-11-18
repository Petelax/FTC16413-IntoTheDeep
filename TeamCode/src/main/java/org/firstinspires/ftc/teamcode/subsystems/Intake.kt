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
import kotlin.math.max
import kotlin.math.min

class Intake(hardwareMap: HardwareMap): SubsystemBase() {
    private var left: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_LEFT)
    private var right: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_RIGHT)
    private var color: RevColorSensorV3 = hardwareMap.get(RevColorSensorV3::class.java, "color")

    private var red = 0.0
    private var green = 0.0
    private var blue = 0.0
    private var max = 0.0

    private var distance = 0.0

    private var lastSpeed = 0.0

    init {
        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.direction = DcMotorSimple.Direction.REVERSE
        color.initialize()
    }

    override fun periodic() {
        red = color.red().toDouble()
        green = color.green().toDouble()
        blue = color.blue().toDouble()

        max = max(red, max(green, blue))

        red /= max
        blue /= max
        green /= max

        distance = color.getDistance(DistanceUnit.MM)
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

    fun getGamePiece(): Sample {
        if (distance >= 15.0) {
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
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            left.power = corrected
            right.power = corrected
            lastSpeed = corrected
        }
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
