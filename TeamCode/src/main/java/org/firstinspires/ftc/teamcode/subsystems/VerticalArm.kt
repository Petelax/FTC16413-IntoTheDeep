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
import kotlin.math.abs

object VerticalArm : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)
    private var cachedPosition = VerticalConstants.VerticalArmPositions.INTAKE
    private var isPowered = false

    private val servo by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(ServoImplEx::class.java, DeviceIDs.VERTICAL_ARM)
    }

    override fun preUserInitHook(opMode: Wrapper) {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        //cachedPosition = -100.0
        isPowered = false
        //servo.position = VerticalConstants.VerticalArmPositions.INTAKE
    }

    fun setPosition(position: Double) {
        val corrected = position.coerceIn(0.0..1.0)
        if (Cache.shouldUpdate(cachedPosition, corrected) || !isPowered) {
            isPowered = true
            servo.position = corrected
            cachedPosition = corrected
        }
    }

    fun kill(): Lambda {
        return Lambda("kill-vert-arm").addRequirements(VerticalArm)
            .setInit{ servo.setPwmDisable() }
    }

    fun getPosition(): Double {
        return cachedPosition
    }

    fun timeNeeded(deltaPosition: Double): Double {
        return abs(deltaPosition * VerticalConstants.VerticalArmConstants.unitsToSeconds)
    }

    fun isArmIntake(): Boolean {
        return cachedPosition < 0.5
    }

    fun isArmSpecimen(): Boolean {
        return cachedPosition > 0.9
    }

    fun sample(): Lambda {
        return Lambda("vertical-arm-sample").addRequirements(VerticalArm)
            .setInit{
                setPosition(VerticalConstants.VerticalArmPositions.SAMPLE)
            }
            .setFinish{true}
    }

    fun specimen(): Lambda {
        return Lambda("vertical-arm-specimen").addRequirements(VerticalArm)
            .setInit{
                setPosition(VerticalConstants.VerticalArmPositions.SPECIMEN)
            }
            .setFinish{true}
    }

    fun intake(): Lambda {
        return Lambda("vertical-arm-intake").addRequirements(VerticalArm)
            .setInit{
                setPosition(VerticalConstants.VerticalArmPositions.INTAKE)
            }
            .setFinish{true}
    }

    fun autoStart(): Lambda {
        return Lambda("vertical-arm-auto-start").addRequirements(VerticalArm)
            .setInit{
                setPosition(VerticalConstants.VerticalArmPositions.AUTO_START)
            }
    }

    fun setVerticalArm(position: Double): Lambda {
        return Lambda("vertical-arm-set").addRequirements(VerticalArm)
            .setInit{ setPosition(position)}
            .setFinish{true}
    }

}
