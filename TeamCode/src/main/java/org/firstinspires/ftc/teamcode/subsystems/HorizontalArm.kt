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
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.Cache
import java.lang.annotation.Inherited

object HorizontalArm : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)
    private var cachedPosition = 100.0

    private val servo by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(ServoImplEx::class.java, DeviceIDs.HORIZONTAL_ARM)
    }

    override fun preUserInitHook(opMode: Wrapper) {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.position = HorizontalConstants.HorizontalArmPositions.IN
    }

    fun setPosition(position: Double) {
        val corrected = position.coerceIn(0.0..1.0)
        if (Cache.shouldUpdate(cachedPosition, corrected)) {
            servo.position = corrected
            cachedPosition = corrected
        }
    }

    fun up() {
        setPosition(HorizontalConstants.HorizontalArmPositions.IN)
    }

    fun down() {
        setPosition(HorizontalConstants.HorizontalArmPositions.OUT)
    }

    fun inHorizontalArm(): Lambda {
        return Lambda("horizontal-arm-in").addRequirements(HorizontalArm)
            .setInit{ up() }
            .setFinish{true}
    }

    fun outHorizontalArm(): Lambda {
        return Lambda("horizontal-arm-out").addRequirements(HorizontalArm)
            .setInit{ down() }
            .setFinish{true}
    }

    fun setHorizontalArm(position: Double): Lambda {
        return Lambda("horizontal-arm-set").addRequirements(HorizontalArm)
            .setInit{ setPosition(position)}
            .setFinish{true}
    }

}
