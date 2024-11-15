package org.firstinspires.ftc.teamcode.commands.drivebase

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow

class FieldCentricDrive(drivebase: SwerveDrivetrain, /*gamepad: GamepadEx*/
                        strafeSpeed: DoubleSupplier,
                        forwardSpeed: DoubleSupplier,
                        turnSpeed: DoubleSupplier,
                        halfSpeed: BooleanSupplier,
                        fieldCentric: BooleanSupplier

): Command() {
    private var drivebase: SwerveDrivetrain
    private var strafeSpeed: DoubleSupplier
    private var forwardSpeed: DoubleSupplier
    private var turnSpeed: DoubleSupplier
    private var halfSpeed: BooleanSupplier
    private var fieldCentric: BooleanSupplier

    /*
    private var gamepad: GamepadEx
     */

    init {
        this.drivebase = drivebase
        this.strafeSpeed = strafeSpeed
        this.forwardSpeed = forwardSpeed
        this.turnSpeed = turnSpeed
        this.halfSpeed = halfSpeed
        this.fieldCentric = fieldCentric
        //this.gamepad = gamepad

    }

    override fun execute() {
        drivebase.firstOrderFieldCentricDrive(
            ChassisSpeeds(
            -forwardSpeed.asDouble.pow(1) * DrivebaseConstants.Measurements.MAX_VELOCITY,
            strafeSpeed.asDouble.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            turnSpeed.asDouble.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
            )
        )
    }

    override fun finished(): Boolean {
        return false
    }

    override fun initialise() {
        TODO("Not yet implemented")
    }

    override fun toString(): String {
        return "field-centric-drive"
    }

    override val requirements: Set<Any> = setOf(drivebase)

    override val runStates: Set<Wrapper.OpModeState>
        get() = TODO("Not yet implemented")

    override fun end(interrupted: Boolean) {

    }

}
