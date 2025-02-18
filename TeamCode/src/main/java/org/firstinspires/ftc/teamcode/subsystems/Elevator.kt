package org.firstinspires.ftc.teamcode.subsystems

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
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import java.lang.annotation.Inherited
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.max

object Elevator : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    private var motorLeft by subsystemCell{
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_LEFT)
    }
    private var motorRight by subsystemCell{
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_RIGHT)
    }
    private var motorClimb by subsystemCell{
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, DeviceIDs.CLIMB)
    }

    private var limit by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.touchSensor.get(DeviceIDs.VERTICAL_LIMIT)
    }

    //private var elevator: MotorGroup
    private var currentPosition: Double = 0.0
    private var currentClimbPosition: Double = 0.0
    private var positionOffset = 0.0
    private var lastSpeed = 0.0
    private var lastClimbSpeed = 0.0
    private var atBottom = true
    private var lastAtBottom = false
    private var currentLeft = 0.0
    private var currentRight = 0.0
    private var speed = 0.0
    //val c = VerticalConstants.ElevatorCoefficients
    //var pidfController = PIDFController(c.KP, c.KI, c.KD, c.KF)

    private val positions = VerticalConstants.ElevatorPositions
    private val coefficients = VerticalConstants.ElevatorCoefficients
    private val constants = VerticalConstants.ElevatorConstants

    var targetPosition = 0.0
        private set

    var targetClimbPosition = 0.0
        private set

    var limitless = false

    lateinit var controller: DoubleController
    lateinit var climbController: DoubleController

    override fun preUserInitHook(opMode: Wrapper) {
        if (opMode.meta.flavor == OpModeMeta.Flavor.AUTONOMOUS) {
            motorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motorClimb.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        motorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorClimb.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorClimb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorRight.direction = DcMotorSimple.Direction.REVERSE
        motorClimb.direction = DcMotorSimple.Direction.REVERSE

        //pidfController.setPIDF(c.KP, c.KI, c.KD, c.KF)

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
                        MotionComponents.STATE -> VerticalConstants.ElevatorConstants.POSITION_TOLERANCE
                        MotionComponents.VELOCITY -> VerticalConstants.ElevatorConstants.VELOCITY_TOLERANCE
                        else -> Double.NaN
                    }
                }
            ),
            outputConsumer = ::setSpeed,
            controllerCalculation = DoubleComponent.P(MotionComponents.STATE, VerticalConstants.ElevatorCoefficients.KP)
                .plus(DoubleComponent.D(MotionComponents.STATE, VerticalConstants.ElevatorCoefficients.KD)),
        )

        climbController = DoubleController(
            targetSupplier = MotionComponentSupplier {
                if (it == MotionComponents.STATE) {
                    return@MotionComponentSupplier targetClimbPosition
                }
                0.0
            },
            stateSupplier = { getClimbPosition() },
            toleranceEpsilon = CachedMotionComponentSupplier(
                MotionComponentSupplier {
                    return@MotionComponentSupplier when (it) {
                        MotionComponents.STATE -> VerticalConstants.ElevatorConstants.CLIMB_POSITION_TOLERANCE
                        MotionComponents.VELOCITY -> VerticalConstants.ElevatorConstants.CLIMB_VELOCITY_TOLERANCE
                        else -> Double.NaN
                    }
                }
            ),
            outputConsumer = ::setClimbSpeed,
            controllerCalculation = DoubleComponent.P(MotionComponents.STATE, VerticalConstants.ClimbCoefficients.KP)
                .plus(DoubleComponent.D(MotionComponents.STATE, VerticalConstants.ClimbCoefficients.KD)),
        )

        targetPosition = 0.0
        controller.enabled = false
        climbController.enabled = false

        defaultCommand = driveAndClimb({-opMode.opMode.gamepad2.left_stick_y.toDouble() + if(opMode.opMode.gamepad1.right_bumper) {-1000.0} else {0.0}}, {if (opMode.opMode.gamepad1.dpad_up) {1.0} else {0.0} + if (opMode.opMode.gamepad1.dpad_down) {-1.0} else {0.0}})
    }

    override fun preUserStartHook(opMode: Wrapper) {
        if (limit.isPressed) {
            positionOffset += currentPosition
        }
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        currentPosition = (motorLeft.currentPosition * constants.TICKS_TO_INCHES) - positionOffset
        currentClimbPosition = motorClimb.currentPosition * constants.CLIMB_TICKS_TO_INCHES

        atBottom = limit.isPressed
        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition
        }
        lastAtBottom = atBottom

    }

    fun reset() {
        currentPosition = 0.0
        positionOffset = 0.0
        controller.controllerCalculation.reset()
        climbController.controllerCalculation.reset()
    }

    /*
    val fsm: StateMachine<States> = StateMachine(States.MANUAL)
        .withState(States.MANUAL) { state: RefCell<States>, name: String ->
            Lambda("nothing").addRequirements(Elevator)
                .setInit{
                    controller.enabled = true
                }
        }
        .withState(States.PID) { state: RefCell<States>, name: String ->
            Lambda("pid").addRequirements(Elevator)
                .setInit{
                    controller.enabled = false
                    controller.controllerCalculation.reset()
                }
        }
     */

    fun driveAndClimb(speed: DoubleSupplier, climbSpeed: DoubleSupplier): Lambda {
        return Lambda("elavator-default-climb").addRequirements(Elevator)
            .setInit{
                controller.enabled = false
                setSpeed(speed.asDouble)
            }
            .setExecute{
                if (abs(climbSpeed.asDouble) > 0.2) {
                    setClimbSpeed(climbSpeed.asDouble)
                } else {
                    setClimbSpeed(0.0)
                }
            }
            .setInterruptible(true)

    }

    fun drive(speed: DoubleSupplier): Lambda {
        return Lambda("elevator-default").addRequirements(Elevator)
            .setInit{
                controller.enabled = false
                setSpeed(speed.asDouble)
                //fsm.schedule(States.MANUAL)
            }
            //.setExecute{setSpeed(speed.asDouble)}
            //.setFinish{false}
            .setInterruptible(true)
            //.setEnd{ interrupted -> if(!interrupted) {setSpeed(0.0)} }
    }

    fun disableController(): Lambda {
        return Lambda("cancel").addRequirements(Elevator)
            .setInit{
                controller.enabled = false
            }
    }

    //fun pid(setPoint: Double): Lambda {return Lambda("hi")}

    /*
    fun pid(setPoint: Double): StatefulLambda<RefCell<PIDFController>> {
        return StatefulLambda("elevator-pid",
            RefCell(PIDFController(
                VerticalConstants.ElevatorCoefficients.KP,
                VerticalConstants.ElevatorCoefficients.KI,
                VerticalConstants.ElevatorCoefficients.KD,
                VerticalConstants.ElevatorCoefficients.KF))).addRequirements(Elevator)
            .setInit { state -> state.get().calculate(getPosition(), setPoint)}
            .setExecute { state -> setSpeed(state.get().calculate(getPosition(), setPoint))}
            .setFinish {state -> if(setPoint <= VerticalConstants.ElevatorPositions.LOWER_LIMIT) { atBottom() } else {state.get().atSetPoint()}}
            //.setEnd {interrupted -> setSpeed((0.0)) }
    }

     */

    fun pid(setPoint: Double): Lambda {
        return Lambda("elevator-pid").addRequirements(Elevator)
            .setInit{
                targetPosition = setPoint
                controller.controllerCalculation.reset()
                controller.enabled = true
            }
            .setInterruptible(true)
    }

    fun pidAuto(setPoint: Double): Lambda {
        return Lambda("elevator-pid-auto").addRequirements(Elevator)
            .setInit{
                targetPosition = setPoint
                controller.controllerCalculation.reset()
                controller.enabled = true
            }
            .setFinish{ atSetPoint() }
            .setInterruptible(true)
    }

    fun pidAutoTimeout(setPoint: Double, timeout: Double): Race {
        return Race(null,
            pidAuto(setPoint),
            Wait(timeout)
        )
    }

    fun pidLimitless(setPoint: Double): Lambda {
        return Lambda("elevator-pid").addRequirements(Elevator)
            .setInit{
                targetPosition = setPoint
                controller.controllerCalculation.reset()
                controller.enabled = true
                limitless = true
            }
            .setInterruptible(true)
    }

    fun waitUntilAboveArm(): Lambda {
        return Lambda("waiting-for-arm")
            .setFinish{ getPosition()>=VerticalConstants.ElevatorPositions.ARM }
    }

    fun waitUntilSetPoint(setPoint: Double): Lambda {
        return Lambda("waiting-for-setpoint")
            .setInit{targetPosition=setPoint}
            .setFinish{ atSetPoint() }
    }

    fun waitUntilSetPoint(): Lambda {
        return Lambda("waiting-for-setpoint").setFinish{ atSetPoint() }
    }

    fun climb(speed: DoubleSupplier) : Lambda {
        return Lambda("elevator-climb").addRequirements(Elevator)
            .setInit{ defaultCommand = null; controller.enabled = false }
            .setExecute{setRawSpeed(speed.asDouble); setClimbSpeed(speed.asDouble)}
            .setFinish{false}
            .setInterruptible(false)
    }

    fun cancel(): Lambda {
        return Lambda("cancel").addRequirements(Elevator)
    }

    fun atSetPoint(): Boolean {
        return abs(targetPosition - getPosition()) <= VerticalConstants.ElevatorConstants.POSITION_TOLERANCE
                && abs(controller.velocity) <= VerticalConstants.ElevatorConstants.VELOCITY_TOLERANCE
    }

    /*
    fun pidCalculate(sp: Double): Double {
        return pidfController.calculate(getPosition(), sp)
    }
     */

    /*
    fun pid(setPoint: Double): Lambda {
        return Lambda("elevator-pid").addRequirements(Elevator)
            .setInit {
                controller.reset()
                pidCalculate(setPoint)
            }
            .setExecute { setSpeed(pidCalculate(setPoint))}
            .setFinish {if(setPoint <= VerticalConstants.ElevatorPositions.LOWER_LIMIT) { atBottom() } else {controller.atSetPoint()}}
            //.setEnd { setSpeed((0.0)) }
    }

     */


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
            if (limitless) {
                setRawSpeed( speed )
            } else {
                setRawSpeed(
                    speed + coefficients.KG + (coefficients.KE * max( (currentPosition - 5.0), 0.0 ))
                )
            }
        }

    }

    fun setClimbSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastClimbSpeed, corrected)) {
            motorClimb.power = corrected
            lastClimbSpeed = corrected
        }
    }

    fun getClimbPosition(): Double {
        return currentClimbPosition
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

    fun getCurrentLeft(): Double {
        return currentLeft
    }
    fun getCurrentRight(): Double {
        return currentRight
    }
    fun getCurrent(): Double {
        return currentRight + currentLeft
    }

}