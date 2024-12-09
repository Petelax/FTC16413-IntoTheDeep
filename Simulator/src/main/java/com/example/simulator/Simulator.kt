package com.example.simulator

import com.example.simulator.geometry.Rotation2d
import com.example.simulator.geometry.Translation2d
import com.example.simulator.geometry.Vector2d
import javafx.application.Application
import javafx.application.Application.launch
import javafx.scene.Scene
import javafx.scene.image.Image
import javafx.scene.image.ImageView
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Circle
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

}

fun main(args: Array<String>) {
    println("test")

    val results = lineCircleIntersection(Vector2d(12.0, 0.0), 3.0, Vector2d(0.0, 0.0), Vector2d(48.0, 0.0))

    results.forEachIndexed { i, point ->
        println("x " + i + ": "+ point.x)
        println("y " + i + ": "+ point.y)
    }

    //launch(Simulator::class.java)

}


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

    results.forEach { t ->
        val point = E + (d * t)
        points.add(point)
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