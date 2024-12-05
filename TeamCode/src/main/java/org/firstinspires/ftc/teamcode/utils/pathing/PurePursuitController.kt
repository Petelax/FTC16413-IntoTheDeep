package org.firstinspires.ftc.teamcode.utils.pathing

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.utils.MathUtil
import org.firstinspires.ftc.teamcode.utils.PIDController
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

object PurePursuitController: Feature {
    override var dependency: Dependency<*> = Dependency { opMode: Wrapper, resolvedFeatures: List<Feature>, yielding: Boolean ->  }
    val turnPID = PIDController(
        DrivebaseConstants.PIDToPosition.RotationKP,
        DrivebaseConstants.PIDToPosition.RotationKI,
        DrivebaseConstants.PIDToPosition.RotationKD
    )
    init {
        turnPID.enableContinuousInput(-PI, PI)
        turnPID.setTolerance(DrivebaseConstants.PIDToPosition.RotationPositionTolerance, DrivebaseConstants.PIDToPosition.RotationVelocityTolerance)
    }

    fun goToPosition(currentPose: Pose2d, x: Double, y: Double, movementSpeed: Double, targetAngle: Double, turnSpeed: Double): ChassisSpeeds {
        val distancetoTarget = hypot(x-currentPose.x, y-currentPose.y)
        val absoluteAngleToTarget = atan2(y-currentPose.y, x-currentPose.y)
        val relativeAngleToTarget = MathUtil.angleModulus(absoluteAngleToTarget - currentPose.rotation.radians)

        val relativeXToTarget = cos(relativeAngleToTarget) * distancetoTarget
        val relativeYToTarget = sin(relativeAngleToTarget) * distancetoTarget

        val xPower = relativeXToTarget / (relativeXToTarget.absoluteValue + relativeYToTarget.absoluteValue)
        val yPower = relativeYToTarget / (relativeXToTarget.absoluteValue + relativeYToTarget.absoluteValue)

        val turnPower = turnPID.calculate(currentPose.heading, targetAngle) * turnSpeed

        return ChassisSpeeds(xPower*DrivebaseConstants.Measurements.MAX_VELOCITY, yPower*DrivebaseConstants.Measurements.MAX_VELOCITY, turnPower)
    }

    fun lineCircleIntersection(circleCenter: Translation2d, radius: Double, linePoint1: Translation2d, linePoint2: Translation2d): List<Translation2d> {
        var lineStart = linePoint1
        var lineEnd = linePoint2

        if (abs(lineStart.y - lineEnd.y) < 0.001) {
            lineStart = Translation2d(lineStart.x, (lineEnd.y + 0.001))
        }
        if (abs(lineStart.x - lineEnd.x) < 0.001) {
            lineStart = Translation2d((lineEnd.x + 0.001), lineStart.y)
        }

        val m = (lineEnd.y - lineStart.y)/(lineEnd.x - lineStart.x)

        val y1 = lineStart.y - circleCenter.y
        val r = radius

        val denominator = m.pow(2) + 1.0

        val term0 = m * y1

        val term1 = sqrt( r.pow(2) + m.pow(2) * r.pow(2) - y1.pow(2) )

        val xRoot0 = -1.0 * (term0 + term1) / (denominator) + circleCenter.x
        val yRoot0 = m * (xRoot0) + y1 + circleCenter.y

        val xRoot1 = -1.0 * (term0 + term1) / (denominator) + circleCenter.x
        val yRoot1 = m * (xRoot1) + y1 + circleCenter.y

        val roots: MutableList<Translation2d> = mutableListOf()

        if (!xRoot0.isNaN()) {
            roots.add(Translation2d(xRoot0, yRoot0))
        }
        if (!xRoot1.isNaN()) {
            roots.add(Translation2d(xRoot1, yRoot1))
        }

        if (roots.isEmpty()) {
            return emptyList()
        }

        val minX = min(lineStart.x, lineEnd.x)
        val maxX = max(lineStart.x, lineEnd.x)

        var goodRoots : MutableList<Translation2d> = mutableListOf()

        roots.forEach { root ->
            if (root.x in minX..maxX) {
                goodRoots.add(root)
            }
        }

        return goodRoots

    }


}