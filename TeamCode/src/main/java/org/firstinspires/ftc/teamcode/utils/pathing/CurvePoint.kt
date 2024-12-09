package org.firstinspires.ftc.teamcode.utils.pathing

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.geometry.Vector2d

data class CurvePoint(
    var pose: Pose2d,
    var moveSpeed: Double,
    var turnSpeed: Double,
    var followDistance: Double,
    var pointLength: Double,
    var slowDownRadians: Double,
    var slowDownAmount: Double
) {
    constructor(pose: Pose2d, moveSpeed: Double, turnSpeed: Double, followDistance: Double) : this(
        pose,
        moveSpeed,
        turnSpeed,
        followDistance,
        0.0,
        0.0,
        0.0
    )

    fun getVector2d(): Vector2d {
        return Vector2d(pose.x, pose.y)
    }

    fun getTranslation2d(): Translation2d {
        return pose.translation

    }

    fun setTranslation2d(translation2d: Translation2d) {
        pose = Pose2d(translation2d, pose.rotation)
    }

}