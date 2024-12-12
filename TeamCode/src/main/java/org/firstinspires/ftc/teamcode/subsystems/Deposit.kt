package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import org.firstinspires.ftc.teamcode.utils.Telemetry
import java.lang.annotation.Inherited
import kotlin.math.max

object Deposit : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)
    private var cachedPosition = 100.0

    private val servo by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(ServoImplEx::class.java, DeviceIDs.DEPOSIT)
    }

    private val crf by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.analogInput.get(DeviceIDs.DEPOSIT_SENSOR)
    }

    private var distance = 100.0
    private var maxVoltage = 3.3

    override fun preUserInitHook(opMode: Wrapper) {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        cachedPosition = 100.0
        servo.position = VerticalConstants.DepositPositions.OUT
        maxVoltage = crf.maxVoltage
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        distance = crf.voltage / maxVoltage * 100.0
        Telemetry.put("deposit distance", getDistance())
        Telemetry.put("really holding", reallyHoldingPiece())
    }

    fun kill(): Lambda {
        return Lambda("deposit-kill").addRequirements(Deposit)
            .setInit{servo.setPwmDisable() }
    }

    fun setPosition(position: Double) {
        val corrected = position.coerceIn(0.0..1.0)
        if (Cache.shouldUpdate(cachedPosition, corrected)) {
            servo.position = corrected
            cachedPosition = corrected
        }
    }

    fun close(): Lambda {
        return Lambda("deposit-in").addRequirements(Deposit)
            .setInit{
                setPosition(VerticalConstants.DepositPositions.IN)
            }
            .setFinish{true}
    }

    fun halfClose(): Lambda {
        return Lambda("deposit-in").addRequirements(Deposit)
            .setInit{
                setPosition(VerticalConstants.DepositPositions.MID)
            }
            .setFinish{true}
    }

    fun open(): Lambda {
        return Lambda("deposit-out").addRequirements(Deposit)
            .setInit{
                setPosition(VerticalConstants.DepositPositions.OUT)
            }
            .setFinish{true}
    }

    fun setDeposit(position: Double): Lambda {
        return Lambda("deposit-set").addRequirements(Deposit)
            .setInit{ setPosition(position)}
            .setFinish{true}
    }

    /**
     * distance from crf on deposit
     */
    fun getDistance() : Double {
        return distance
    }

    fun holdingPiece() : Boolean {
        return getDistance() <= VerticalConstants.DepositPositions.DEPOSIT_THRESHOLD
    }

    fun reallyHoldingPiece() : Boolean {
        return getDistance() <= VerticalConstants.DepositPositions.TIGHT_DEPOSIT_THRESHOLD
    }


}
