package com.example.simulator

import javafx.application.Application
import javafx.application.Application.launch
import javafx.scene.Scene
import javafx.scene.image.Image
import javafx.scene.image.ImageView
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import javafx.scene.shape.Circle
import javafx.stage.Stage

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
    launch(Simulator::class.java)
}
