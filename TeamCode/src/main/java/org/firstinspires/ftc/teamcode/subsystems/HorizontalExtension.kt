package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import java.lang.annotation.Inherited
import java.util.function.DoubleSupplier

object HorizontalExtension : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    private var motor by subsystemCell{
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_LEFT)
    }

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

    override fun preUserInitHook(opMode: Wrapper) {
        motor = opMode.opMode.hardwareMap.get(DcMotorEx::class.java, DeviceIDs.HORIZONTAL_EXTENSION)

        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        defaultCommand = drive{-opMode.opMode.gamepad2.right_stick_y.toDouble()}
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        currentPosition = (motor.currentPosition * constants.TICKS_TO_INCHES) - positionOffset
    }

    fun drive(speed: DoubleSupplier): Lambda {
        return Lambda("horizontal-extension-default").addRequirements(HorizontalExtension)
            .setExecute{setSpeed(speed.asDouble)}
            .setFinish{false}
            .setInterruptible(true)
            .setEnd{ interrupted -> if(!interrupted) {setSpeed(0.0)} }
    }

    fun setRawSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
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
