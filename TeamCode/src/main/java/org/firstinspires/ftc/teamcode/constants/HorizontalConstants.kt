package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

class HorizontalConstants {
    @Config
    object HorizontalExtensionCoefficients {
        @JvmField var KS = 0.0
        @JvmField var KV = 0.0
        @JvmField var KA = 0.0

        @JvmField var KP = 0.4
        @JvmField var KI = 0.0
        @JvmField var KD = 0.006
        @JvmField var KF = 0.0

        @JvmField var KG = 0.0
    }

    @Config
    object HorizontalExtensionPositions {
        @JvmField var UPPER_LIMIT = 13.8
        @JvmField var LOWER_LIMIT = 0.1
        @JvmField var TOP = 13.5
        @JvmField var BOTTOM = 0.2
    }

    @Config
    object HorizontalExtensionConstants {
        @JvmField var TICKS_TO_INCHES = 0.04487179487
        @JvmField var POSITION_TOLERANCE = 0.5
        @JvmField var VELOCITY_TOLERANCE = 5.0
    }

    @Config
    object IntakeSpeeds {
        @JvmField var MAX = 1.0
        @JvmField var STOP = 0.0
    }

    @Config
    object HorizontalArmPositions {
        @JvmField var OUT = 0.07
        @JvmField var IN = 1.0
    }

    /**
     * horizontal arm
     * angle 135 units
     * pwm power 74.9%
     *
     * TODO:
     * horizontal wrist
     * angle 85 units
     * 110deg
     * pwm power 100%
     *
     * TODO: intake
     */



}