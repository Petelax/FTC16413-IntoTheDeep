package org.firstinspires.ftc.teamcode.commands.drivebase

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

class ProfiledPIDToPosition(private val drive: SwerveDrivetrain, private val setpoint: Pose2d, private val endSpeeds: ChassisSpeeds): CommandBase() {
    val c = DrivebaseConstants.PIDToPosition
    val xController: ProfiledPIDController = ProfiledPIDController(c.TranslationKP, c.TranslationKI, c.TranslationKD, TrapezoidProfile.Constraints(c.MaxVelocity, c.MaxAcceleration))
    val yController: ProfiledPIDController = ProfiledPIDController(c.TranslationKP, c.TranslationKI, c.TranslationKD, TrapezoidProfile.Constraints(c.MaxVelocity, c.MaxAcceleration))
    val headingController: PIDController = PIDController(c.RotationKP, c.RotationKI, c.RotationKD)

    init {
        headingController.enableContinuousInput(-PI, PI)

        xController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        yController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        headingController.setTolerance(c.RotationPositionTolerance, c.RotationVelocityTolerance)
        addRequirements(drive)
    }

    override fun initialize() {
        xController.goal = TrapezoidProfile.State(setpoint.x, endSpeeds.vxMetersPerSecond)
        yController.goal = TrapezoidProfile.State(setpoint.y, endSpeeds.vyMetersPerSecond)
        headingController.setpoint = setpoint.rotation.radians
    }

    override fun execute() {
        val currentPose = drive.getPose()
        var xFeedback = -xController.calculate(currentPose.x)
        xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
        var yFeedback = -yController.calculate(currentPose.y)
        yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
        var headingFeedback = -headingController.calculate(currentPose.rotation.radians)
        headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF

        drive.firstOrderFieldCentricDrive(ChassisSpeeds(xFeedback, yFeedback, headingFeedback))

    }

    override fun isFinished(): Boolean {
        return xController.atGoal() && yController.atGoal() && headingController.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    private fun clipRange(n: Double, m: Double): Double {
        return if (abs(n) > abs(m)) {
            m * n.sign
        } else {
            n
        }
    }

}
