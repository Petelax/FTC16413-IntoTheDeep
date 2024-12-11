package com.example.simulator

import com.example.simulator.geometry.Pose2d
import com.example.simulator.geometry.Translation2d
import com.example.simulator.geometry.Vector2d

/**
 * @param pose pose
 * @param moveSpeed translation speed from 0.0 to 1.0
 * @param turnSpeed translation speed from 0.0 to 1.0
 * @param followDistance distance to look ahead and follow in inches
 * @param targetSpeed speed to drive based on curvature
 */
data class CurvePoint(
    var pose: Pose2d,
    var moveSpeed: Double,
    var turnSpeed: Double,
    var followDistance: Double,
    var targetSpeed: Double,
    var totalDistance: Double
) {
    constructor() : this(
        Pose2d(),
        1.0,
        1.0,
        0.0,
        1.0,
        0.0
    )

    constructor(
        pose: Pose2d,
        moveSpeed: Double,
        turnSpeed: Double,
        followDistance: Double,
    ) : this(
        pose,
        moveSpeed,
        turnSpeed,
        followDistance,
        moveSpeed,
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
