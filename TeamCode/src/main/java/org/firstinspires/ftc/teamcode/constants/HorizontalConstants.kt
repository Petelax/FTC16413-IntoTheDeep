package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

class HorizontalConstants {
    @Config
    object HorizontalExtensionCoefficients {
        @JvmField var KS = 0.0
        @JvmField var KV = 0.0
        @JvmField var KA = 0.0

        @JvmField var KP = 0.16
        @JvmField var KI = 0.0
        @JvmField var KD = 0.001
        @JvmField var KF = 0.07

        @JvmField var KG = 0.0
    }

    @Config
    object HorizontalExtensionPositions {
        @JvmField var UPPER_LIMIT = 13.8
        @JvmField var LOWER_LIMIT = -1.0
        @JvmField var TOP = 13.5
        @JvmField var BOTTOM = 0.1
        @JvmField var INSIDE = 0.5
        @JvmField var BOTTOM_HOLD = 0.04
        @JvmField var CLEAR = 4.25
    }

    @Config
    object HorizontalExtensionConstants {
        @JvmField var TICKS_TO_INCHES = 0.04487179487
        @JvmField var POSITION_TOLERANCE = 0.5
        @JvmField var VELOCITY_TOLERANCE = 5.0

        @JvmField var MINIMUM_SPEED = 0.01
        @JvmField var RETRACT_KEEP_SPEED = -0.20
        @JvmField var RETRACTING_SPEED = -0.70
    }

    @Config
    object IntakeSpeeds {
        @JvmField var MAX = 1.0
        @JvmField var STOP = 0.0
        @JvmField var BACK = -0.1
        @JvmField var BACK_BACK = -0.1
        @JvmField var BACK_TIME = 0.050
        @JvmField var BACK_BACK_TIME = 0.150
        @JvmField var PRE_BACK_TIME = 0.0
    }

    @Config
    object HorizontalArmPositions {
        @JvmField var OUT = 0.075 //0.07
        @JvmField var IN = 1.0
        @JvmField var MID = 0.5
    }

    @Config
    object HorizontalWristPositions {
        @JvmField var IN = 1.0
        @JvmField var OUT = 0.1
    }

    /**
     * horizontal arm
     * angle 135 units
     * pwm power 74.9%
     *
     * horizontal wrist
     * angle 85 units
     * 110deg
     * pwm power 74.9%
     * ulta high sens
     *
     * intake left
     * pwm power 100.0%
     * no inversion
     *
     * intake right
     * pwm power 100.0%
     * no inversion
     *
     */



}