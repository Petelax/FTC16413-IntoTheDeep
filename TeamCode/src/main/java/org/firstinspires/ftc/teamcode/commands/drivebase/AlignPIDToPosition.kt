package org.firstinspires.ftc.teamcode.commands.drivebase

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.atan2
import kotlin.math.sign

class AlignPIDToPosition(private val drive: SwerveDrivetrain, private val setpoint: Pose2d): SequentialCommandGroup() {
    init {
        val c = DrivebaseConstants.PIDToPosition
        val xController: PIDFController = PIDFController(c.TranslationKP, c.TranslationKI, c.TranslationKD, 0.0)
        val yController: PIDFController = PIDFController(c.TranslationKP, c.TranslationKI, c.TranslationKD, 0.0)
        val headingController: PIDController = PIDController(c.RotationKP, c.RotationKI, c.RotationKD)
        xController.setPoint = setpoint.x
        yController.setPoint = setpoint.y
        headingController.setpoint = setpoint.rotation.radians

        val currentPose = drive.getPose()
        var xFeedback = -xController.calculate(currentPose.x)
        xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
        var yFeedback = -yController.calculate(currentPose.y)
        yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
        var headingFeedback = -headingController.calculate(currentPose.rotation.radians)
        headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF

        val desiredSpeeds = ChassisSpeeds(xFeedback, yFeedback, headingFeedback)

        addCommands(
            AlignModules(drive, desiredSpeeds).withTimeout(800),
            PIDToPosition(drive, setpoint)
        )
        addRequirements(drive)
    }

}