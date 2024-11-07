package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.abs
import kotlin.math.sign

class AlignModules(private val drive: SwerveDrivetrain, private val heading: Rotation2d): CommandBase() {
    private val desiredHeading: Rotation2d = heading
    init {

        addRequirements(drive)
    }

    override fun initialize() {
        drive.setModuleHeadings(desiredHeading, desiredHeading, desiredHeading, desiredHeading)
    }

    override fun execute() {
        drive.setModuleHeadings(desiredHeading, desiredHeading, desiredHeading, desiredHeading)
    }

    override fun isFinished(): Boolean {
        return drive.areModulesAligned(desiredHeading, 10.0)
    }

    override fun end(interrupted: Boolean) {

    }

}