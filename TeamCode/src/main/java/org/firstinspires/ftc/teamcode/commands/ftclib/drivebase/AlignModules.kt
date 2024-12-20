package org.firstinspires.ftc.teamcode.commands.ftclib.drivebase

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.subsystems.ftclib.swerve.SwerveDrivetrain

class AlignModules(private val drive: SwerveDrivetrain, private val speeds: ChassisSpeeds): CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun initialize() {
        drive.setModuleHeadings(speeds)
    }

    override fun execute() {
        drive.setModuleHeadings(speeds)
    }

    override fun isFinished(): Boolean {
        return drive.areModulesAligned()
    }

    override fun end(interrupted: Boolean) {

    }

}