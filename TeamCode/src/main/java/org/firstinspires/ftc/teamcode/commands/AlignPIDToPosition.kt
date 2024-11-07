package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import kotlin.math.atan2

class AlignPIDToPosition(private val drive: SwerveDrivetrain, private val pose: Pose2d): SequentialCommandGroup() {

    init {
        val deltaPose = pose.minus(drive.getPose())
        val desiredHeading = atan2( deltaPose.translation.y, deltaPose.translation.x )

        addCommands(
            AlignModules(drive, Rotation2d(desiredHeading)),
            PIDToPosition(drive, pose)
        )
        addRequirements(drive)
    }
}