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

object VerticalWrist : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)
    private var cachedPosition = 100.0

    private val servo by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(ServoImplEx::class.java, DeviceIDs.VERTICAL_WRIST)
    }

    override fun preUserInitHook(opMode: Wrapper) {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        cachedPosition = 100.0
        servo.position = VerticalConstants.VerticalWristPositions.INTAKE
    }

    fun setPosition(position: Double) {
        val corrected = position.coerceIn(0.0..1.0)
        if (Cache.shouldUpdate(cachedPosition, corrected)) {
            servo.position = corrected
            cachedPosition = corrected
        }
    }

    fun kill(): Lambda {
        return Lambda("vertical-wrist-kill").addRequirements(VerticalWrist)
            .setInit{servo.setPwmDisable() }
    }

    fun sample(): Lambda {
        return Lambda("vertical-wrist-sample").addRequirements(VerticalWrist)
            .setInit{
                setPosition(VerticalConstants.VerticalWristPositions.SAMPLE)
            }
            .setFinish{true}
    }

    fun specimenPlace(): Lambda {
        return Lambda("vertical-wrist-specimen").addRequirements(VerticalWrist)
            .setInit{
                setPosition(VerticalConstants.VerticalWristPositions.SPECIMEN_PLACE)
            }
            .setFinish{true}
    }

    fun specimenPickup(): Lambda {
        return Lambda("vertical-wrist-specimen").addRequirements(VerticalWrist)
            .setInit{
                setPosition(VerticalConstants.VerticalWristPositions.SPECIMEN_PICKUP)
            }
            .setFinish{true}
    }

    fun intake(): Lambda {
        return Lambda("vertical-wrist-intake").addRequirements(VerticalWrist)
            .setInit{
                setPosition(VerticalConstants.VerticalWristPositions.INTAKE)
            }
            .setFinish{true}
    }

    fun setVerticalWrist(position: Double): Lambda {
        return Lambda("vertical-wrist-set").addRequirements(VerticalWrist)
            .setInit{ setPosition(position)}
            .setFinish{true}
    }

}
