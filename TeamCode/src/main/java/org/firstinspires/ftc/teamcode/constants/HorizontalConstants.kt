package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

class HorizontalConstants {
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
        @JvmField var UPPER_LIMIT = 27.5
        @JvmField var LOWER_LIMIT = 0.1
        @JvmField var TOP = 27.0
        @JvmField var BOTTOM = 0.20
    }

    @Config
    object ElevatorConstants {
        @JvmField var TICKS_TO_INCHES = 0.0005784991162
        @JvmField var POSITION_TOLERANCE = 0.25
        @JvmField var VELOCITY_TOLERANCE = 5.0
    }

    @Config
    object IntakeSpeeds {
        @JvmField var MAX = 1.0
        @JvmField var STOP = 0.0
    }



    /**
     * horizontal arm
     * angle 135 units
     * pwm power 100%
     *
     * horizontal wrist
     * angle 85 units
     * pwm power 100%
     */



}