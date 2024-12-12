package org.firstinspires.ftc.teamcode.utils

import kotlin.math.absoluteValue

class RateLimiter(var rate: Double) {
    private var lastOutput = 0.0
    private var lastTime = System.nanoTime()

    fun calculate(input: Double) : Double {
        val currentTimestamp = System.nanoTime()
        val deltaTime = (currentTimestamp - lastTime) / 1E9
        val maxChange = rate * deltaTime

        var output = lastOutput + (input - lastOutput).coerceIn(-maxChange, maxChange)
        if (output.absoluteValue < lastOutput) {
            output = input
        }

        lastTime = currentTimestamp
        lastOutput = output

        return output
    }

    fun reset() {
        lastOutput = 0.0
    }
}