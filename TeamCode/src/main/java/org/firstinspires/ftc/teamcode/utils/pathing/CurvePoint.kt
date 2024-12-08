package org.firstinspires.ftc.teamcode.utils.pathing

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Translation2d

class CurvePoint() {
    fun toTranslation2d(): Translation2d {
        return pose.translation

    }

    fun setTranslation2d(translation2d: Translation2d) {
        pose = Pose2d(translation2d, pose.rotation)
    }

    var pose = Pose2d()
    var moveSpeed: Double = 0.0
    var turnSpeed: Double = 0.0
    var followDistance: Double = 0.0
    var pointLength: Double = 0.0
    var slowDownRadians: Double = 0.0
    var slowDownAmount: Double = 0.0

    constructor(
        pose: Pose2d,
        moveSpeed: Double,
        turnSpeed: Double,
        followDistance: Double,
        pointLength: Double,
        slowDownRadians: Double,
        slowDownAmount: Double
    ) : this() {
        this.pose = pose
        this.moveSpeed = moveSpeed
        this.turnSpeed = turnSpeed
        this.followDistance = followDistance
        this.pointLength = pointLength
        this.slowDownRadians = slowDownRadians
        this.slowDownAmount = slowDownAmount
    }

    constructor(curvePoint: CurvePoint) : this() {
        pose = curvePoint.pose
        moveSpeed = curvePoint.moveSpeed
        turnSpeed = curvePoint.turnSpeed
        followDistance = curvePoint.followDistance
        pointLength = curvePoint.pointLength
        slowDownRadians = curvePoint.slowDownRadians
        slowDownAmount = curvePoint.slowDownAmount
    }

}