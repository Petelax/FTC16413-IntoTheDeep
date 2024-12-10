package org.firstinspires.ftc.teamcode.utils.pathing

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.geometry.Vector2d

/**
 * @param pose pose
 * @param moveSpeed translation speed from 0.0 to 1.0
 * @param turnSpeed translation speed from 0.0 to 1.0
 * @param followDistance distance to look ahead and follow in inches
 */
data class CurvePoint(
    var pose: Pose2d,
    var moveSpeed: Double,
    var turnSpeed: Double,
    var followDistance: Double,
) {
    constructor() : this(
        Pose2d(),
        1.0,
        1.0,
        0.0,
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