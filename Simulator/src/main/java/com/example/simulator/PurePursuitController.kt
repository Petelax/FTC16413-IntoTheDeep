package com.example.simulator

import com.example.simulator.geometry.Pose2d
import com.example.simulator.geometry.Rotation2d
import com.example.simulator.geometry.Vector2d
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
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

    /**
     * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
     *
     * BigO: Order N x M
     * @param arr
     * @return
     */
    fun doubleArrayCopy(arr: Array<DoubleArray>): Array<DoubleArray> {
        //size first dimension of array

        val temp = Array(arr.size) {
            DoubleArray(
                arr[0].size
            )
        }

        for (i in arr.indices) {
            //Resize second dimension of array
            temp[i] = DoubleArray(arr[i].size)

            //Copy Contents
            for (j in arr[i].indices) temp[i][j] = arr[i][j]
        }

        return temp
    }

    /**
     * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
     * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
     * converge. If this happens, try increasing the tolerance level.
     *
     * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met.
     *
     * @param curvePoints
     * @param weight_data
     * @param weight_smooth
     * @param tolerance
     * @return
     */
    fun smoother(
        //path: Array<DoubleArray>,
        curvePoints: List<CurvePoint>,
        weight_data: Double,
        weight_smooth: Double,
        tolerance: Double
    ): List<CurvePoint> {
        //copy array
        val path = pathToDoubleArray(curvePoints)

        val newPath: Array<DoubleArray> = doubleArrayCopy(path)

        var change = tolerance
        while (change >= tolerance) {
            change = 0.0
            for (i in 1 until path.size - 1) for (j in path[i].indices) {
                val aux = newPath[i][j]
                newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]))
                change += abs(aux - newPath[i][j])
            }
        }

        return doubleArrayToPath(curvePoints, newPath)
    }

    fun pathToDoubleArray(path: List<CurvePoint>) : Array<DoubleArray> {
        var array = Array(path.size) {DoubleArray(2)}
        path.forEachIndexed { i, point ->
            array[i][0] = point.pose.x
            array[i][1] = point.pose.y
        }
        return array
    }

    fun doubleArrayToPath(curvePoints: List<CurvePoint>, arr: Array<DoubleArray>) : List<CurvePoint> {
        var newPath: MutableList<CurvePoint> = mutableListOf()
        curvePoints.forEachIndexed { i, point ->
            newPath.add(point.copy(pose = Pose2d(arr[i][0], arr[i][1], point.pose.rotation)))
        }

        return newPath
    }

    /**
     * @return curvature
     */
    fun getCurvature(p: Vector2d, q: Vector2d, r: Vector2d) : Double {
        //val x1 = p.x
        val y1 = p.y

        val x2 = q.x
        val y2 = q.y

        val x3 = r.x
        val y3 = r.y

        val x1 = if (p.x == x2) { p.x+0.001 } else { p.x }

        val k1 = 0.5 * (x1*x1 + y1*y1 - x2*x2 - y2*y2)/(x1 - x2)
        val k2 = (y1 - y2)/(x1 - x2)
        val b = 0.5 * (x2*x2 - 2 * x2 * k1 + y2*y2 - x3*x3 + 2 * x3 * k1 - y3*y3)/(x3 * k2 - y3 + y2 - x2 * k2)
        val a = k1 - k2 * b
        val radius = sqrt((x1 - a).pow(2) + (y1 - b).pow(2))
        val curvature = 1/radius

        return curvature
    }

    /**
     * path must have greater or equal to 3 points
     */
    fun setPathTargetSpeed(path: List<CurvePoint>, k: Double) : List<CurvePoint> {
        assert(path.size>=3)

        var newPath: MutableList<CurvePoint> = mutableListOf(path[0].copy(targetSpeed = path[0].moveSpeed))

        for (i in 1 until path.size-1) {
            val p: CurvePoint = path[i]
            val r: CurvePoint = path[i+1]
            val q: CurvePoint = path[i-1]

            val curvature = getCurvature(p.getVector2d(), q.getVector2d(), r.getVector2d())

            val targetSpeed = min(p.moveSpeed, k / curvature)
            //val radius = 0.1/curvature
            //println("targetSpeed: $radius")

            val followDistance = targetSpeed * 10.0

            newPath.add(i, path[i].copy(followDistance=followDistance, targetSpeed=targetSpeed))

        }

        newPath.add(path.last().copy(targetSpeed = path.last().moveSpeed))

        return newPath

    }

    fun distancePoints(curvePoints: List<CurvePoint>) : List<CurvePoint> {
        var newPath: MutableList<CurvePoint> = mutableListOf()

        newPath.add(0, curvePoints[0].copy(totalDistance = 0.0))

        var distance = 0.0
        for (i in 1 until curvePoints.size) {
            val a = curvePoints[i].getVector2d()
            val b = curvePoints[i-1].getVector2d()

            distance += distance(a, b)
            newPath.add(curvePoints[i].copy(totalDistance = distance))
        }

        return newPath
    }

    fun distance(a: Vector2d, b: Vector2d): Double {
        return a.minus(b).magnitude()
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

    fun waypointsToPath(points: List<CurvePoint>, spacing: Double = 1.0, kSmooth: Double = 0.8, kCurvature: Double = 5.0) : List<CurvePoint> {
        val injected = injectPoints(points, spacing)
        val a = 1-kSmooth
        val smoothed = smoother(injected, a, kSmooth, 0.001)
        val distanced = distancePoints(smoothed)
        val constrained = setPathTargetSpeed(distanced, kCurvature)
        return constrained
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