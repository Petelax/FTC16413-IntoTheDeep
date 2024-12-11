package org.firstinspires.ftc.teamcode.utils

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.DrivetrainPIDCoefficients
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.sign

class DrivetrainPIDController(private val c: DrivetrainPIDCoefficients) {
    private val xController: PIDFController = PIDFController(c.TranslationKP, c.TranslationKI, c.TranslationKD, 0.0)
    private val yController: PIDFController = PIDFController(c.TranslationKP, c.TranslationKI, c.TranslationKD, 0.0)
    private val headingController = PIDController(c.RotationKP, c.RotationKI, c.RotationKD)

    init{
        headingController.enableContinuousInput(-PI, PI)
        headingController.setTolerance(c.RotationPositionTolerance, c.RotationVelocityTolerance)
        xController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        yController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
    }

    fun calculate(currentPose: Pose2d, targetPose: Pose2d, movementSpeed: Double, turnSpeed: Double) : ChassisSpeeds {
        var xFeedback = -xController.calculate(currentPose.x, targetPose.x)
        xFeedback += xFeedback.sign * c.KF
        xFeedback /= DrivebaseConstants.Measurements.MAX_VELOCITY
        var yFeedback = -yController.calculate(currentPose.y, targetPose.y)
        yFeedback += yFeedback.sign * c.KF
        yFeedback /= DrivebaseConstants.Measurements.MAX_VELOCITY
        var headingFeedback = -headingController.calculate(currentPose.rotation.radians, targetPose.rotation.radians)
        headingFeedback += headingFeedback.sign * c.KF

        if (hypot(xFeedback, yFeedback) > movementSpeed) {
            xFeedback /= (xFeedback.absoluteValue + yFeedback.absoluteValue)
            yFeedback /= (xFeedback.absoluteValue + yFeedback.absoluteValue)
        }

        return ChassisSpeeds(xFeedback* DrivebaseConstants.Measurements.MAX_VELOCITY*movementSpeed, yFeedback*DrivebaseConstants.Measurements.MAX_VELOCITY*movementSpeed, headingFeedback*turnSpeed)
    }

    fun calculateNormalized(currentPose: Pose2d, targetPose: Pose2d, movementSpeed: Double, turnSpeed: Double) : ChassisSpeeds {
        var xFeedback = -xController.calculate(currentPose.x, targetPose.x)
        xFeedback += xFeedback.sign * c.KF
        xFeedback /= DrivebaseConstants.Measurements.MAX_VELOCITY
        var yFeedback = -yController.calculate(currentPose.y, targetPose.y)
        yFeedback += yFeedback.sign * c.KF
        yFeedback /= DrivebaseConstants.Measurements.MAX_VELOCITY
        var headingFeedback = -headingController.calculate(currentPose.rotation.radians, targetPose.rotation.radians)
        headingFeedback += headingFeedback.sign * c.KF

        xFeedback /= (xFeedback.absoluteValue + yFeedback.absoluteValue)
        yFeedback /= (xFeedback.absoluteValue + yFeedback.absoluteValue)

        return ChassisSpeeds(xFeedback* DrivebaseConstants.Measurements.MAX_VELOCITY*movementSpeed, yFeedback*DrivebaseConstants.Measurements.MAX_VELOCITY*movementSpeed, headingFeedback*turnSpeed)
    }


    fun reset() {
        xController.reset()
        yController.reset()
        headingController.reset()
    }


}