package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.TouchSensor
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import java.lang.annotation.Inherited
import java.util.function.DoubleSupplier

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

    private var limit by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.touchSensor.get(DeviceIDs.VERTICAL_LIMIT)
    }

    //private var elevator: MotorGroup
    private var currentPosition: Double = 0.0
    private var positionOffset = 0.0
    private var lastSpeed = 0.0
    private var atBottom = true
    private var lastAtBottom = false
    private var currentLeft = 0.0
    private var currentRight = 0.0
    private var speed = 0.0

    private val positions = VerticalConstants.ElevatorPositions
    private val coefficients = VerticalConstants.ElevatorCoefficients
    private val constants = VerticalConstants.ElevatorConstants

    override fun preUserInitHook(opMode: Wrapper) {
        motorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        motorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorRight.direction = DcMotorSimple.Direction.REVERSE

        defaultCommand = drive{-opMode.opMode.gamepad2.left_stick_y.toDouble()}
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        currentPosition = (motorLeft.currentPosition * constants.TICKS_TO_INCHES) - positionOffset

        atBottom = limit.isPressed
        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition
        }
        lastAtBottom = atBottom

    }

    fun drive(speed: DoubleSupplier): Lambda {
        return Lambda("elevator-default").addRequirements(Elevator)
            .setExecute{setSpeed(speed.asDouble)}
            .setFinish{false}
            .setInterruptible(true)
            .setEnd{ interrupted -> if(!interrupted) {setSpeed(0.0)} }
    }

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
            .setEnd {interrupted -> { setSpeed((0.0)) }}
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