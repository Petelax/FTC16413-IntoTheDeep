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
import java.lang.annotation.Inherited

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

    override fun preUserInitHook(opMode: Wrapper) {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.position = VerticalConstants.DepositPositions.OUT
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

}
