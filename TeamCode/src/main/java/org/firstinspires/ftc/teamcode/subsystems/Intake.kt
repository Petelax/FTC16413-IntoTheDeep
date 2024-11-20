package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl
import dev.frozenmilk.dairy.cachinghardware.CachingCRServo
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

object Intake : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    private var cachedPower = 100.0

    private val left by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_LEFT)
    }
    private val right by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_RIGHT)
    }

    override fun preUserInitHook(opMode: Wrapper) {
        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        left.power = 0.0
        right.power = 0.0
        right.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setPower(power: Double) {
        val corrected = power.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(cachedPower, corrected)) {
            left.power = corrected
            right.power = corrected
            cachedPower = corrected
        }
    }

    fun stopIntake(): Lambda {
        return Lambda("intake-stop").addRequirements(Intake)
            .setInit{ setPower(0.0) }
            .setFinish { true }
    }

    fun runIntake(): Lambda {
        return Lambda("intake-run").addRequirements(Intake)
            .setInit{ setPower(HorizontalConstants.IntakeSpeeds.MAX) }
            .setFinish { true }
    }

}