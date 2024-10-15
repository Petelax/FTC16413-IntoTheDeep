package org.firstinspires.ftc.teamcode.drive.commands

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.pow

class FieldCentricDrive(drivebase: SwerveDrivetrain, /*gamepad: GamepadEx*/
                        strafeSpeed: DoubleSupplier,
                        forwardSpeed: DoubleSupplier,
                        turnSpeed: DoubleSupplier,
                        halfSpeed: BooleanSupplier,
                        fieldCentric: BooleanSupplier

): CommandBase() {
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

        addRequirements(drivebase)
    }

    override fun initialize() {
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

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return false
    }

}
