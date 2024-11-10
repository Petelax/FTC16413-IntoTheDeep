package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import kotlin.math.abs

object Cache {
    fun shouldUpdate(lastValue: Double, currentValue: Double, tolerance: Double): Boolean {
        return (abs(currentValue - lastValue) >= tolerance || (currentValue == 0.0 && lastValue != 0.0) || (currentValue >= 1.0 && !(lastValue >= 1.0)) || (currentValue <= -1.0 && !(lastValue <= -1.0)) || lastValue.isNaN())
    }

    fun shouldUpdate(lastValue: Double, currentValue: Double): Boolean {
        return shouldUpdate(lastValue, currentValue, DrivebaseConstants.Measurements.cachingTolerance)
    }
}