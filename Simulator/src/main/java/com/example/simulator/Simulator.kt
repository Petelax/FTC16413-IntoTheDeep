package com.example.simulator

import com.example.simulator.geometry.Pose2d
import com.example.simulator.geometry.Rotation2d
import com.example.simulator.geometry.Translation2d
import com.example.simulator.geometry.Vector2d
import javafx.application.Application
import javafx.application.Application.launch
import javafx.application.Platform
import javafx.scene.Scene
import javafx.scene.image.Image
import javafx.scene.image.ImageView
import javafx.scene.layout.Background
import javafx.scene.layout.BackgroundFill
import javafx.scene.layout.Pane
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Circle
import javafx.scene.shape.Line
import javafx.stage.Stage
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

class Simulator : Application() {
    private val points = listOf(
        Pair(12.0, 12.0),
        Pair(72.0, 24.0),
        Pair(104.0, 80.0),
    )
    /*
    override fun start(primaryStage: Stage) {
        val field = Image(javaClass.getResourceAsStream("/images/field.png"))

        val fieldView = ImageView(field)
        //imageView.fitWidth = 1000.0
        //imageView.fitHeight = 1000.0

        val root = StackPane()
        root.children.add(fieldView)

        val initialWidth = 500.0
        val initialHeight = 500.0

        val pointCircles = points.map { (x, y) ->
            Circle(0.0, 0.0, 10.0).apply {
                fill = Color.RED
                setPointPosition(x, y, initialWidth, initialHeight)
            }
        }

        root.children.addAll(pointCircles)

        val test = Circle(100.0, 100.0, 10.0)
        test.fill = Color.GREEN

        root.children.add(test)

        fieldView.fitWidth = initialWidth
        fieldView.fitHeight = initialHeight
        fieldView.isPreserveRatio = true

        val scene = Scene(root, initialWidth, initialHeight)


        scene.widthProperty().addListener { _, _, newWidth ->
            fieldView.fitWidth = newWidth.toDouble()
            val newHeight = newWidth.toDouble() * field.height/field.width
            fieldView.fitHeight = newHeight

            pointCircles.forEach {
                it.setPointPosition(it.centerX, it.centerY, newWidth.toDouble(), newHeight)
            }
        }

        scene.heightProperty().addListener { _, _, newHeight ->
            fieldView.fitHeight = newHeight.toDouble()
            val newWidth = newHeight.toDouble() * field.width/field.height
            fieldView.fitWidth = newWidth

            pointCircles.forEach {
                it.setPointPosition(it.centerX, it.centerY, newWidth, newHeight.toDouble())
            }
        }

        primaryStage.title = "FTC Simulator"
        primaryStage.scene = scene
        primaryStage.show()
    }

    private fun Circle.setPointPosition(x: Double, y: Double, width: Double, height: Double) {
        this.centerX = (x / 144.0) * width
        this.centerY = (y / 144.0) * height
    }

     */
    override fun start(primaryStage: Stage) {
        // Create a pane to hold everything
        val pane = Pane()

        pane.background = Background(BackgroundFill(Color.color(0.16, 0.16, 0.16), null, null))

        // Load the background image
        val bgImage = Image(javaClass.getResourceAsStream("/images/field.png"))  // Replace with the path to your image
        val imageView = ImageView(bgImage)

        // Set the fixed size of the image to match the desired graph area
        imageView.fitWidth = 1080.0
        imageView.fitHeight = 1080.0
        imageView.rotate = -90.0
        imageView.opacity = 0.8

        // Add the background image to the pane
        pane.children.add(imageView)

        // Plot the points (example points)
        /*
        val points = listOf(Pair(100.0, 200.0), Pair(300.0, 400.0), Pair(500.0, 100.0), Pair(700.0, 500.0), Pair(540.0, 540.0))
        for (point in points) {
            val circle = Circle(point.first, point.second, 10.0)  // Create a circle at the given point
            circle.fill = Color.RED  // Set the color of the point
            pane.children.add(circle)  // Add the point to the pane
        }

         */

        /*
        val newPath = PurePursuitController.injectPoints(path, 2.0)

        val b = 0.85
        val newerPath = PurePursuitController.smoother(newPath, 0.2, 0.8, 0.001)
        val constrainedPath = PurePursuitController.setPathTargetSpeed(newerPath, 0.05)
        constrainedPath.forEach { point ->
            println(point.toString())
        }

         */
        /*
        val newPath = PurePursuitController.waypointsToPath(path, kCurvature = 0.1)

        newPath.forEach { point ->
            println(point)
        }

         */

        val newPath = sampleTest
        newPath.forEach { point ->
            println(point)
        }

        graphPath(pane, newPath)

        /*
        graphPath(pane, first)
        graphPath(pane, second)
        graphPath(pane, third)
        graphPath(pane, fourth)
        graphPath(pane, fifth)
        graphPath(pane, sixth)
        graphPath(pane, seventh)
        graphPath(pane, eighth)

         */

        //graphPath(pane, seventh)
        //val arr = PurePursuitController.pathToDoubleArray(first)

        // Create the scene with the fixed size
        val scene = Scene(pane, 1080.0, 1080.0)

        scene.setOnKeyPressed { event ->
            if (event.code.toString() == "Q") {
                Platform.exit()  // This will exit the application
            }
        }

        // Set up the stage
        primaryStage.title = "FTC Simulator"
        primaryStage.scene = scene
        primaryStage.show()
    }

    /*
    private val first = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(29.75, 7.375, Rotation2d.fromDegrees(180.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(27.31, 10.54, Rotation2d.fromDegrees(180.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
    ), kSmooth = 0.95)

     */
    private val sampleTest = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(10.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
    ))

    private val first = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 24.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 25.5, Rotation2d.fromDegrees(90.0)), 0.4, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 35.0, Rotation2d.fromDegrees(90.0)), 0.2, 1.0, 6.0),
    ))


    private val second = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(78.0, 36.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 34.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 30.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 25.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(82.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(87.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(90.0, 25.0, Rotation2d.fromDegrees(180.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 30.0, Rotation2d.fromDegrees(-95.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(109.0, 60.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 60.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 25.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.30, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 11.75, Rotation2d.fromDegrees(-90.0)), 0.30, 1.0, 6.0),
    ), kSmooth = 0.895, kCurvature = 0.080, spacing = 1.5)

    private val third = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.8, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(118.19, 18.60, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(116.10, 19.55, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(73.0, 21.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(73.0, 35.75, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
    ), kSmooth = 0.95, kCurvature = 0.08)

    private val fourth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(73.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.65, 1.0, 5.0),
        CurvePoint(Pose2d(73.0, 26.0, Rotation2d.fromDegrees(90.0)), 0.65, 1.0, 5.0),
        CurvePoint(Pose2d(82.0, 26.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(90.50, 29.05, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(100.0, 32.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(108.0, 40.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(110.0, 45.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(118.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(132.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(132.0, 20.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(132.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.8, 1.0, 6.0),
    ), kSmooth = 0.95, kCurvature = 0.075)

    private val fifth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(132.0, 16.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(132.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(118.19, 20.5, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(116.10, 21.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 24.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 35.75, Rotation2d.fromDegrees(90.0)), 0.95, 1.0, 5.0),
    ), kSmooth = 0.95, kCurvature = 0.08)

    private val sixth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(70.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 30.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 23.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(85.0, 24.0, Rotation2d.fromDegrees(170.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(95.0, 24.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(100.0, 24.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 25.0, Rotation2d.fromDegrees(-90.0)), 0.8, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.75, 1.0, 6.0),
    ), kSmooth = 0.95, kCurvature = 0.08)

    private val seventh = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(118.19, 20.5, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(116.10, 21.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(65.0, 22.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(65.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
    ), kSmooth = 0.95, kCurvature = 0.075)

    private val eighth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(65.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(65.0, 23.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(100.0, 10.0, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 6.0),
    ), kSmooth = 0.95, kCurvature = 0.08)

    fun graphPath(pane: Pane, path: List<CurvePoint>) {
        for (point in path) {
            val pose = point.pose
            val circle = Circle(toPx(((72.0-pose.y))+72.0), toPx(((72.0-pose.x))+72.0), toPx(1.0))  // Create a circle at the given point
            //val circle = Circle(toPx(pose.y), toPx(pose.x), toPx(2.0))  // Create a circle at the given point
            //val circle1 = Circle(540.0, 540.0, 10.0)  // Create a circle at the given point
            circle.fill = Color.color(0.73, 0.02, 0.99) //Color.PURPLE  // Set the color of the point
            //circle1.fill = Color.PURPLE  // Set the color of the point
            //pane.children.add(circle)  // Add the point to the pane
            //pane.children.add(circle1)
        }

        for (i in 0 until path.size - 1) {
            val start = toVector(path[i].pose)
            val end = toVector(path[i + 1].pose)

            // Create a line from the current point to the next point
            val line = Line(start.x, start.y, end.x, end.y)
            line.stroke = Color.color(0.73, 0.02, 0.99)
            line.strokeWidth = 3.0  // Set the thickness of the line
            pane.children.add(line)

        }

    }

    fun toVector(pose: Pose2d) : Vector2d {
        return Vector2d(
            toPx(((72.0-pose.y))+72.0), toPx(((72.0-pose.x))+72.0)
        )
    }

    fun toPx(n: Double) : Double {
        return (n/144.0) * 1080.0
    }


}

fun main(args: Array<String>) {
    /*
    println("targets: ")

    val results = lineCircleIntersection(Vector2d(12.0, 0.0), 3.0, Vector2d(0.0, 0.0), Vector2d(48.0, 0.0))

    results.forEachIndexed { i, point ->
        println("x " + i + ": "+ point.x)
        println("y " + i + ": "+ point.y)
    }

     */
    launch(Simulator::class.java)

}

fun distance(a: Vector2d, b: Vector2d): Double {
    val deltaX = b.x - a.x
    val deltaY = b.y - a.y
    val distance = sqrt(deltaX*deltaX + deltaY*deltaY)
    return distance
}


/**
 * @param C center of circle
 * @param r circle radius
 * @param E starting point of line
 * @param L ending point of line
 */
fun lineCircleIntersection(C: Vector2d, r: Double, E: Vector2d, L: Vector2d): List<Vector2d> {
    val d = L - E
    val f = E - C

    val a = d.dot(d)
    val b = 2*f.dot(d)
    val c = f.dot(f) - r*r
    var discriminant = b*b - 4*a*c

    var results: MutableList<Double> = mutableListOf()

    if (discriminant < 0) {
        println("no intersection")
        return emptyList()
    } else {
        discriminant = sqrt(discriminant)
        val t1 = (-b - discriminant)/(2*a)
        val t2 = (-b + discriminant)/(2*a)

        if (t1 in 0.0..1.0) {
            results.add(t1)
        }
        if (t2 in 0.0..1.0) {
            results.add(t2)
        }

    }

    var points: MutableList<Vector2d> = emptyList<Vector2d>().toMutableList()

    val maxX = max(E.x, L.x)
    val minX = min(E.x, L.x)
    val maxY = max(E.y, L.y)
    val minY = min(E.y, L.y)

    results.forEach { t ->
        val point = E + (d * t)
        if (point.x in minX..maxX && point.y in minY..maxY) {
            points.add(point)
        }
    }

    return points

}


/*
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

    var xRoot0 = -1.0 * (term0 + term1) / (denominator)
    val yRoot0 = m * (xRoot0) + y1 + circleCenter.y
    xRoot0 += circleCenter.x

    var xRoot1 = -1.0 * (term0 + term1) / (denominator)
    val yRoot1 = m * (xRoot1) + y1 + circleCenter.y
    xRoot1 += circleCenter.x

    val roots: MutableList<Translation2d> = mutableListOf()

    if (!xRoot0.isNaN()) {
        val root = Translation2d(xRoot0, yRoot0)
        println("x root 0: $xRoot0")
        println("y root 0: $yRoot0")
        roots.add(root)
    }
    if (!xRoot1.isNaN()) {
        val root = Translation2d(xRoot1, yRoot1)
        println("x root 1: $xRoot1")
        println("y root 1: $yRoot1")
        roots.add(root)
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

    //val vec: Vector2d = Vector2d()

    return goodRoots

}

*/