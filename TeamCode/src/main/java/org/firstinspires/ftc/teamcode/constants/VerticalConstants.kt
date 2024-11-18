package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

class VerticalConstants {

    @Config
    object ElevatorCoefficients {
        @JvmField var KS = 0.0
        @JvmField var KV = 0.0
        @JvmField var KA = 0.0

        @JvmField var KP = 0.3
        @JvmField var KI = 0.0
        @JvmField var KD = 0.006
        @JvmField var KF = 0.0

        @JvmField var KG = 0.042
    }

    @Config
    object ElevatorPositions {
        @JvmField var UPPER_LIMIT = 27.5 //28.2
        @JvmField var LOWER_LIMIT = 0.1 //28.2
        @JvmField var TOP = 27.0 //28.2
        @JvmField var BOTTOM = 0.20
    }

    @Config
    object ElevatorConstants {
        @JvmField var TICKS_TO_INCHES = 0.0005784991162
        @JvmField var POSITION_TOLERANCE = 0.25
        @JvmField var VELOCITY_TOLERANCE = 5.0
    }

    @Config
    object VerticalArmPositions {
        @JvmField var INTAKE = 0.0
        @JvmField var SAMPLE = 0.6
        @JvmField var SPECIMEN = 0.93
    }

    @Config
    object DepositPositions {
        @JvmField var IN = 0.0
        @JvmField var OUT = 1.00
    }

    @Config
    object VerticalWristPositions {
        @JvmField var ONE = 0.02
        @JvmField var TWO = 0.45
        @JvmField var THREE = 0.65
        @JvmField var FOUR = 0.935
    }

    /**
     * vertical arm
     * angle 210 units
     * pwm power 74.9%
     * sensitivity ultra high
     *
     * vertical wrist
     * angle 135 units
     * pwm power 74.9%
     *
     * deposit
     * angle 50 units
     * pwm power 94.1%
     * sensitivity high
     */

}