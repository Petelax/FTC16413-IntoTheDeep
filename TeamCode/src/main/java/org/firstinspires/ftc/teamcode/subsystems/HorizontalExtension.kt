package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent
import dev.frozenmilk.dairy.core.util.controller.implementation.DoubleController
import dev.frozenmilk.dairy.core.util.supplier.numeric.CachedMotionComponentSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponentSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import java.lang.annotation.Inherited
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

object HorizontalExtension : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    private var motor by subsystemCell{
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, DeviceIDs.HORIZONTAL_EXTENSION)
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

    private var targetPosition = 0.0
    lateinit var controller: DoubleController
    private var holdPosition = false
    private var lastHoldPosition = true

    override fun preUserInitHook(opMode: Wrapper) {
        if (opMode.meta.flavor == OpModeMeta.Flavor.AUTONOMOUS) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motor.direction = DcMotorSimple.Direction.REVERSE

        controller = DoubleController(
            targetSupplier = MotionComponentSupplier {
                if (it == MotionComponents.STATE) {
                    return@MotionComponentSupplier targetPosition
                }
                0.0
            },
            stateSupplier = {getPosition()},
            toleranceEpsilon = CachedMotionComponentSupplier(
                MotionComponentSupplier {
                    return@MotionComponentSupplier when (it) {
                        MotionComponents.STATE -> HorizontalConstants.HorizontalExtensionConstants.POSITION_TOLERANCE
                        MotionComponents.VELOCITY -> HorizontalConstants.HorizontalExtensionConstants.VELOCITY_TOLERANCE
                        else -> Double.NaN
                    }
                }
            ),
            outputConsumer = ::setSpeed,
            controllerCalculation = DoubleComponent.P(MotionComponents.STATE, HorizontalConstants.HorizontalExtensionCoefficients.KP)
                .plus(DoubleComponent.D(MotionComponents.STATE, HorizontalConstants.HorizontalExtensionCoefficients.KD)),
        )

        targetPosition = 0.0
        controller.enabled = false

        defaultCommand = driveAndStop{-opMode.opMode.gamepad2.right_stick_y.toDouble().pow(3.0)}
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        currentPosition = (motor.currentPosition * constants.TICKS_TO_INCHES) - positionOffset
    }

    fun reset() {
        currentPosition = 0.0
        positionOffset = 0.0
        controller.controllerCalculation.reset()
    }

    fun drive(speed: DoubleSupplier): Lambda {
        return Lambda("horizontal-extension-default").addRequirements(HorizontalExtension)
            .setInit{
                controller.enabled = false
                setSpeed(speed.asDouble)
            }
            .setInterruptible(true)
            //.setEnd{ interrupted -> if(!interrupted) {setSpeed(0.0)} }
    }

    fun spin(speed: Double): Lambda {
        return Lambda("horizontal-spin").addRequirements(HorizontalExtension)
            .setInit{
                controller.enabled = false
                setSpeed(speed)
            }
    }

    fun driveAndStop(speed: DoubleSupplier): Lambda {
        return Lambda("horizontal-extension-god-help-me").addRequirements(HorizontalExtension)
            .setExecute{
                if (holdPosition && speed.asDouble < 0.02) {
                    //targetPosition = HorizontalConstants.HorizontalExtensionPositions.BOTTOM
                    controller.enabled = true
                } else if (holdPosition && speed.asDouble > 0.02) {
                    holdPosition = false
                    controller.enabled = false
                    setSpeed(speed.asDouble)
                } else if (!holdPosition && atBottom()) {
                    holdPosition = true
                    targetPosition = HorizontalConstants.HorizontalExtensionPositions.BOTTOM_HOLD
                    controller.enabled = true
                } else {
                    controller.enabled = false
                    setSpeed(speed.asDouble)
                }

                lastHoldPosition = holdPosition
            }
            .setInterruptible(true)

    }

    fun pid(setPoint: Double): Lambda {
        return Lambda("horizontal-extension-pid").addRequirements(HorizontalExtension)
            .setInit{
                targetPosition = setPoint
                controller.controllerCalculation.reset()
                controller.enabled = true
            }
            .setInterruptible(true)
    }

    fun disableController(): Lambda {
        return Lambda("cancel").addRequirements(HorizontalExtension)
            .setInit{
                controller.enabled = false
                holdPosition = false
            }
    }

    fun waitUntilSetPoint(setPoint: Double): Lambda {
        return Lambda("waiting-for-setpoint")
            .setInit{targetPosition=setPoint}
            .setFinish{ atSetPoint() }
    }

    fun waitUntilSetPoint(): Lambda {
        return Lambda("waiting-for-setpoint").setFinish{ atSetPoint() }
    }

    fun atSetPoint(): Boolean {
        return abs(targetPosition - getPosition()) <= HorizontalConstants.HorizontalExtensionConstants.POSITION_TOLERANCE
                && abs(controller.velocity) <= HorizontalConstants.HorizontalExtensionConstants.VELOCITY_TOLERANCE
    }

    fun kill(): Lambda {
        return Lambda("kill-horizontal-extension").addRequirements(HorizontalExtension)
            .setInit{ motor.setMotorDisable() }
    }

    fun stop(): Lambda {
        return Lambda("stop-horizontal-extension").addRequirements(HorizontalExtension)
            .setInit{ controller.enabled=false; setRawSpeed(0.0) }
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
            val f = if (speed.absoluteValue > constants.MINIMUM) {
                coefficients.KF * sign(speed)
            } else {
                0.0
            }
            setRawSpeed(speed + f)
        }

    }

    fun getPosition(): Double {
        return currentPosition
    }

    fun atBottom(): Boolean {
        //return atBottom
        return getPosition() < 1.0
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
