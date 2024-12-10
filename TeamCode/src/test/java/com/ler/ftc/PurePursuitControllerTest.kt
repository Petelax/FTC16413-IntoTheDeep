package com.ler.ftc

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.utils.pathing.CurvePoint
import org.firstinspires.ftc.teamcode.utils.pathing.PurePursuitController
import org.junit.jupiter.api.AfterAll
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class PurePursuitControllerTest {
    private val pp = PurePursuitController

    private val path = listOf(
        CurvePoint(Pose2d(0.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(48.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(48.0, 48.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(72.0, 48.0, Rotation2d()), 0.2, 0.2, 3.0),
    )

    private val oneTurn = listOf(
        CurvePoint(Pose2d(0.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(48.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(48.0, 12.0, Rotation2d()), 0.2, 0.2, 3.0),
    )

    private val straightPath = listOf(
        CurvePoint(Pose2d(0.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(48.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
    )

    private val v = DrivebaseConstants.Measurements.MAX_VELOCITY

    @Test
    fun testLineCircleIntersection() {
        assertEquals(listOf(Pair(Vector2d(6.0, 0.0), 0.125)), pp.lineCircleIntersection(Vector2d(0.0, 0.0), 6.0, Vector2d(0.0, 0.0), Vector2d(48.0, 0.0)))
    }

    @Test
    fun testGetFollowPathPoint() {
        val actual = pp.getFollowPointPath(path, Pose2d(0.0, 0.0, Rotation2d()), 3.0, CurvePoint(),
            0.0)
        val expected = CurvePoint(Pose2d(3.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)
    }

    @Test
    fun `mid first line`() {
        val actual = pp.getFollowPointPath(path, Pose2d(12.0, 0.0, Rotation2d()), 3.0, CurvePoint(),
            0.25)
        val expected = CurvePoint(Pose2d(15.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)
    }

    @Test
    fun `corner`() {
        val actual = pp.getFollowPointPath(path, Pose2d(48.0, 0.0, Rotation2d()), 3.0,
            CurvePoint(Pose2d(47.5, 0.0, Rotation2d()), 0.2, 0.2, 3.0), 1.0)
        val expected = CurvePoint(Pose2d(48.0, 3.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)
    }

    @Test
    fun `mid second line`() {
        val actual = pp.getFollowPointPath(path, Pose2d(48.0, 12.0, Rotation2d()), 3.0, CurvePoint(), 1.25)
        val expected = CurvePoint(Pose2d(48.0, 15.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)
    }

    @Test
    fun `right before second corner`() {
        val actual = pp.getFollowPointPath(path, Pose2d(48.0, 45.0, Rotation2d()), 3.0,  CurvePoint().copy(pose = Pose2d(48.0, 45.0, Rotation2d())),1.92)
        val expected = CurvePoint(Pose2d(48.0, 48.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)
    }

    @Test
    fun `second corner`() {
        val previous = pp.getFollowPointPath(path, Pose2d(48.0, 45.0, Rotation2d()), 3.0,  CurvePoint(),1.93)
        //println("index: " + previous.second)
        //println("x: " + previous.first.pose.x)
        //println("y: " + previous.first.pose.y)

        val actual = pp.getFollowPointPath(path, Pose2d(48.0, 48.0, Rotation2d()), 3.0,  CurvePoint(),previous.second)
        val expected = CurvePoint(Pose2d(51.0, 48.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)
    }

    @Test
    fun `just after second corner`() {
        val previous = pp.getFollowPointPath(path, Pose2d(48.0, 48.0, Rotation2d()), 3.0,  CurvePoint(),2.0)

        val actual = pp.getFollowPointPath(path, Pose2d(49.0, 48.0, Rotation2d()), 3.0,  CurvePoint(),previous.second)
        val expected = CurvePoint(Pose2d(52.0, 48.0, Rotation2d()), 0.2, 0.2, 3.0)

        assertEquals(expected, actual.first)

    }

    @Test
    fun `testFollowPath`() {
        pp.lastIndex = 0.0
        val actual = pp.followPath(path, Pose2d(0.0, 0.0, Rotation2d()))
        val expected = ChassisSpeeds(-v * path[0].moveSpeed, 0.0, 0.0)

        assert(chassisSpeedsEqual(expected, actual))
    }

    @Test
    fun `horizontal FollowPath`() {
        pp.lastIndex = 0.0
        pp.lastPoint = horizontalPath[0]
        val actual = pp.followPath(horizontalPath, Pose2d(0.0, 0.0, Rotation2d()))
        val expected = ChassisSpeeds(0.0, v * path[0].moveSpeed, 0.0)

        assertEquals(-expected.vxMetersPerSecond, actual.vxMetersPerSecond, 0.001, "x")
        assertEquals(-expected.vyMetersPerSecond, actual.vyMetersPerSecond, 0.001, "y")
        assertEquals(-expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 0.001, "heading")
    }

    private val horizontalPath = listOf(
        CurvePoint(Pose2d(0.0, 0.0, Rotation2d()), 0.2, 0.2, 3.0),
        CurvePoint(Pose2d(0.0, 48.0, Rotation2d()), 0.2, 0.2, 3.0),
    )

    @Test
    fun `FollowPath mid`() {
        pp.lastIndex = 1.2
        pp.lastPoint = path[0]
        val actual = pp.followPath(path, Pose2d(48.0, 12.0, Rotation2d()))
        //println("index: " + pp.lastIndex)
        val expected = ChassisSpeeds(0.0, -v * path[0].moveSpeed, 0.0)
        //println("x: " + actual.vxMetersPerSecond)
        //println("y: " + actual.vyMetersPerSecond)

        assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 0.1, "x")
        assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 0.001, "y")
        assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 0.001, "heading")
    }

    /*
    @Test
    fun `FollowPath end`() {
        pp.lastIndex = 0.999
        val actual = pp.followPath(path, Pose2d(47.5, 0.0, Rotation2d()))
        println("index: " + pp.lastIndex)
        val expected = ChassisSpeeds(-v * path[0].moveSpeed, 0.0, 0.0)
        println("x: " + actual.vxMetersPerSecond)
        println("y: " + actual.vyMetersPerSecond)

        //assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 0.001, "x")
        assertEquals(actual.vxMetersPerSecond < -0.01, "x")
        assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 0.001, "y")
        assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 0.001, "heading")
    }
     */

    @Test
    fun `FollowPath turn`() {
        pp.lastIndex = 0.996
        pp.lastPoint = path[0]
        val actual = pp.followPath(path, Pose2d(48.0, 0.0, Rotation2d()))
        val expected = ChassisSpeeds(0.0, -v * path[0].moveSpeed, 0.0)
        println("index: " + pp.lastIndex)
        println("x: " + actual.vxMetersPerSecond)
        println("y: " + actual.vyMetersPerSecond)

        assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 0.001, "x")
        assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 0.001, "y")
        assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 0.001, "heading")
    }

    @Test
    fun `goToPosition y`() {
        val actual = pp.goToPosition(Pose2d(48.0, 0.0, Rotation2d()), Pose2d(48.0, 3.0, Rotation2d()), 0.2, 0.2)
        val expected = ChassisSpeeds(
            0.0,
            v*0.3,
            0.0
        )

        assertEquals(-expected.vxMetersPerSecond, actual.vxMetersPerSecond, 0.001, "x")
        assertEquals(-expected.vyMetersPerSecond, actual.vyMetersPerSecond, 0.3, "y")
        assertEquals(-expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 0.001, "heading")
    }

    @Test
    fun `injection`() {
        val newPoints = pp.injectPoints(path, 3.0)
        newPoints.forEach { point ->
            println(point)
        }
        val actual = pp.getFollowPointPath(newPoints, Pose2d(), 3.0, path[0], 0.0)
        val expected = Pair(newPoints[1], 1.0)

        assertEquals(expected, actual)

    }


    private fun chassisSpeedsEqual(a: ChassisSpeeds, b: ChassisSpeeds) : Boolean {
        val x = a.vxMetersPerSecond == b.vxMetersPerSecond
        val y = a.vyMetersPerSecond == b.vyMetersPerSecond
        val theta = a.omegaRadiansPerSecond == b.omegaRadiansPerSecond
        return x && y && theta
    }

}