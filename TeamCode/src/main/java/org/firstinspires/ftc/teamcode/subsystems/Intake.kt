package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.PwmControl
import dev.frozenmilk.dairy.cachinghardware.CachingCRServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Cache
import org.firstinspires.ftc.teamcode.utils.Globals
import java.lang.Math.pow
import java.lang.annotation.Inherited
import java.util.concurrent.atomic.LongAdder
import java.util.function.DoubleSupplier
import kotlin.math.max
import kotlin.math.pow

object Intake : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    private var cachedPower = 100.0

    private var red = 0.0
    private var blue = 0.0
    private var max = 0.0
    private var colors = NormalizedRGBA()

    private var distance = 0.0

    private val left by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_LEFT)
    }
    private val right by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_RIGHT)
    }

    private val colour by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(RevColorSensorV3::class.java, DeviceIDs.COLOUR)
    }

    var readColours = true

    override fun preUserInitHook(opMode: Wrapper) {
        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        left.power = 0.0
        right.power = 0.0
        right.direction = DcMotorSimple.Direction.REVERSE

    }

    override fun preUserLoopHook(opMode: Wrapper) {
        if (readColours) {
            colors = colour.normalizedColors

            red = colors.red.toDouble()
            blue = colors.blue.toDouble()
            max = max(red, blue)

            red /= max
            blue /= max

            distance = colour.getDistance(DistanceUnit.MM)

            opMode.opMode.telemetry.addData("game piece", getGamePiece().name)
        }

        defaultCommand = setSpeedSupplier { opMode.opMode.gamepad2.right_trigger.toDouble().pow(3) - opMode.opMode.gamepad2.left_trigger.pow(3) }

    }

    fun kill(): Lambda {
        return Lambda("intake-kill").addRequirements(Intake)
            .setInit{ left.setPwmDisable(); right.setPwmDisable() }
    }

    fun stopColourSensor(): Lambda {
        return Lambda("stop-colour-sensor").setRequirements(Intake)
            .setInit{ readColours = false }
    }

    fun startColourSensor(): Lambda {
        return Lambda("start-colour-sensor").setRequirements(Intake)
            .setInit{ readColours = true }
    }

    private fun setPower(power: Double) {
        val corrected = power.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(cachedPower, corrected)) {
            left.power = corrected
            right.power = corrected
            cachedPower = corrected
        }
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

    fun getRed(): Double {
        return red
    }

    fun getBlue(): Double {
        return blue
    }

    fun stopIntake(): Lambda {
        return Lambda("intake-stop").addRequirements(Intake)
            .setInit{ setPower(0.0) }
            .setFinish { true }
    }

    fun runIntake(): Lambda {
        return Lambda("intake-run").addRequirements(Intake)
            .setInit{ setPower(HorizontalConstants.IntakeSpeeds.MAX) }
            .setFinish { true }
    }

    fun setSpeed(n: Double): Lambda {
        return Lambda("intake-set-speed").addRequirements(Intake)
            .setExecute{ setPower(n) }
            .setFinish{ true }
    }

    fun setSpeedSupplier(n: DoubleSupplier): Lambda {
        return Lambda("intake-set-speed-supplier").addRequirements(Intake)
            .setExecute{ setPower(n.asDouble) }
            //.setFinish{false}
    }

    fun backDrive(): Sequential {
        return Sequential(
            setSpeed(HorizontalConstants.IntakeSpeeds.BACK),
            Wait(HorizontalConstants.IntakeSpeeds.BACK_TIME),
            stopIntake()
        )

    }

    fun runIntakeStopping(): Lambda {
        return Lambda("intake-run-stopping").addRequirements(Intake)
            .setInit{ readColours = true; setPower(HorizontalConstants.IntakeSpeeds.MAX) }
            /*
            .setExecute {
                if (Globals.AllianceColour == AllianceColours.Red) {
                    if (getGamePiece() != Sample.RED && getGamePiece() != Sample.YELLOW) {
                        setPower(HorizontalConstants.IntakeSpeeds.MAX)
                    } else {
                        //backDrive().schedule()
                        setPower(0.0)
                    }
                } else {
                    if (getGamePiece() != Sample.BLUE && getGamePiece() != Sample.YELLOW) {
                        setPower(HorizontalConstants.IntakeSpeeds.MAX)
                    } else {
                        //backDrive().schedule()
                        setPower(0.0)
                    }
                }
            }
             */
            .setFinish {
                if (Globals.AllianceColour == AllianceColours.Red) {
                    getGamePiece() == Sample.RED || getGamePiece() == Sample.YELLOW
                } else {
                    getGamePiece() == Sample.BLUE || getGamePiece() == Sample.YELLOW
                }
            }
            .setEnd{setSpeed(HorizontalConstants.IntakeSpeeds.BACK)}
            .setInterruptible{true}
    }

    enum class Sample {
        NONE,
        YELLOW,
        RED,
        BLUE
    }

}