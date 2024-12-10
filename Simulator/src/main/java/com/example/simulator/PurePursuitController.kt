package com.example.simulator

import com.example.simulator.geometry.Pose2d
import com.example.simulator.geometry.Vector2d
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.ceil
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

object PurePursuitController {
    //override var dependency: Dependency<*> = Dependency { opMode: Wrapper, resolvedFeatures: List<Feature>, yielding: Boolean ->  }

    val path: List<CurvePoint> = listOf()

    var lastIndex = 0.0
    var lastPoint = CurvePoint()



    /**
     * @param E starting point of line
     * @param L ending point of line
     * @param t fractional index
     */
    fun fractionalIndexToPoint(E: Vector2d, L: Vector2d, t: Double) : Vector2d {
        val d = L - E

        return E + (d * t)
    }

    fun injectPoints(points: List<CurvePoint>, spacing: Double) : List<CurvePoint> {
        var newPoints: MutableList<CurvePoint> = mutableListOf()

        for (i in 0 until points.size - 1) {
            val startPoint = points[i].copy()
            val endPoint = points[i+1].copy()
            var vector = endPoint.getVector2d() - startPoint.getVector2d()

            val count = ceil(vector.magnitude() / spacing).toInt()

            vector = vector.normalize() * spacing

            for (j in 0..count-1) {
                val newVector = startPoint.getVector2d() + (vector * j.toDouble())
                newPoints.add(endPoint.copy(pose = Pose2d(newVector.x, newVector.y, endPoint.pose.rotation)))
            }


        }

        newPoints.add(points.last())

        return newPoints

    }

    /**
     * @return first curvepoint, second index
     */
    fun getFollowPointPath(curvePoints: List<CurvePoint>, currentPose: Pose2d, followRadius: Double, lastPose: CurvePoint, lastFoundIndex: Double): Pair<CurvePoint, Double> {
        var intersections: MutableList<Pair<Vector2d, Double>> = mutableListOf()

        for (i in lastFoundIndex.toInt() until curvePoints.size - 1) {
            val startLine = curvePoints[i].copy()
            val endLine = curvePoints[i+1].copy()

            lineCircleIntersection(Vector2d(currentPose.x, currentPose.y), followRadius, startLine.getVector2d(), endLine.getVector2d()).forEach { pair: Pair<Vector2d, Double> ->
                if (pair.second + i > lastFoundIndex) {
                    intersections.add(Pair(pair.first, pair.second+i))
                }
            }

        }

        if (intersections.isNotEmpty()) {
            intersections.sortBy { it.second }
            val pair = intersections[0]
            val nextCurvePoint = curvePoints[ceil(pair.second).toInt()]
            return Pair(nextCurvePoint.copy(pose = Pose2d(pair.first.x, pair.first.y, nextCurvePoint.pose.rotation)), pair.second)
        }

        return Pair(lastPose, lastFoundIndex)

    }

    /*
    fun goToPosition(currentPose: Pose2d, targetPose: Pose2d, movementSpeed: Double, turnSpeed: Double): ChassisSpeeds {
        val x = targetPose.x
        val y = targetPose.y
        val targetAngle = targetPose.rotation.radians
        val distancetoTarget = hypot(x-currentPose.x, y-currentPose.y)
        val absoluteAngleToTarget = atan2(y-currentPose.y, x-currentPose.y)
        val relativeAngleToTarget = MathUtil.angleModulus(absoluteAngleToTarget - currentPose.rotation.radians)

        val relativeXToTarget = cos(relativeAngleToTarget) * distancetoTarget
        val relativeYToTarget = sin(relativeAngleToTarget) * distancetoTarget

        val xPower = relativeXToTarget / (relativeXToTarget.absoluteValue + relativeYToTarget.absoluteValue)
        val yPower = relativeYToTarget / (relativeXToTarget.absoluteValue + relativeYToTarget.absoluteValue)

        val turnPower = turnPID.calculate(currentPose.heading, targetAngle) * turnSpeed

        return ChassisSpeeds(xPower*DrivebaseConstants.Measurements.MAX_VELOCITY*movementSpeed, yPower*DrivebaseConstants.Measurements.MAX_VELOCITY*movementSpeed, turnPower)
    }

     */

    /**
     * @param C center of circle
     * @param r circle radius
     * @param E starting point of line
     * @param L ending point of line
     *
     * @return list of pairs with (point, index)
     */
    fun lineCircleIntersection(C: Vector2d, r: Double, E: Vector2d, L: Vector2d): List<Pair<Vector2d, Double>> {
        val d = L - E
        val f = E - C

        val a = d.dot(d)
        val b = 2*f.dot(d)
        val c = f.dot(f) - r*r
        var discriminant = b*b - 4*a*c

        var indices: MutableList<Double> = mutableListOf()

        if (discriminant < 0) {
            println("no intersection")
            return emptyList()
        } else {
            discriminant = sqrt(discriminant)
            val t1 = (-b - discriminant)/(2*a)
            val t2 = (-b + discriminant)/(2*a)

            if (t1 in 0.0..1.0) {
                indices.add(t1)
            }
            if (t2 in 0.0..1.0) {
                indices.add(t2)
            }

        }


        val maxX = max(E.x, L.x)
        val minX = min(E.x, L.x)
        val maxY = max(E.y, L.y)
        val minY = min(E.y, L.y)

        val results: MutableList<Pair<Vector2d, Double>> = mutableListOf()

        indices.forEach { t ->
            val point = E + (d * t)
            if (point.x in minX..maxX && point.y in minY..maxY) {
                results.add(Pair(point, t))
            }
        }

        return results

    }


}