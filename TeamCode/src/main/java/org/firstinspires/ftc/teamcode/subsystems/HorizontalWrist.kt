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

object HorizontalWrist : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)
    var cachedPosition = HorizontalConstants.HorizontalArmPositions.IN
        private set
    private var isPowered = false

    private val servo by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(ServoImplEx::class.java, DeviceIDs.HORIZONTAL_WRIST)
    }

    override fun preUserInitHook(opMode: Wrapper) {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        //cachedPosition = 100.0
        isPowered = false
        //servo.position = HorizontalConstants.HorizontalWristPositions.IN
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
        return Lambda("horizontal-wrist-kill").addRequirements(HorizontalWrist)
            .setInit{servo.setPwmDisable(); }
    }

    fun up() {
        setPosition(HorizontalConstants.HorizontalWristPositions.IN)
    }

    fun down() {
        setPosition(HorizontalConstants.HorizontalWristPositions.OUT)
    }

    fun inHorizontalWrist(): Lambda {
        return Lambda("horizontal-wrist-in").addRequirements(HorizontalWrist)
            .setInit{up()}
            .setFinish{true}
    }

    fun outHorizontalWrist(): Lambda {
        return Lambda("horizontal-wrist-out").addRequirements(HorizontalWrist)
            .setInit{down()}
            .setFinish{true}
    }

    fun setHorizontalWrist(position: Double): Lambda {
        return Lambda("horizontal-wrist-set").addRequirements(HorizontalWrist)
            .setInit{ setPosition(position)}
            .setFinish{true}
    }

}
