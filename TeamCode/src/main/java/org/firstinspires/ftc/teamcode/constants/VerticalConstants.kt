package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

class VerticalConstants {

    @Config
    object ElevatorCoefficients {
        @JvmField var KS = 0.0
        @JvmField var KV = 0.0
        @JvmField var KA = 0.0

        @JvmField var KP = 0.21//0.30
        @JvmField var KI = 0.0
        @JvmField var KD = 0.004
        @JvmField var KF = 0.0

        @JvmField var KG = 0.042 //0.042
        @JvmField var KE = 0.0027
    }

    @Config
    object ClimbCoefficients {
        @JvmField var KP = 5.00
        @JvmField var KI = 0.0
        @JvmField var KD = 0.0
    }

    @Config
    object ElevatorPositions {
        @JvmField var UPPER_LIMIT = 28.1 //28.2
        @JvmField var LOWER_LIMIT = 0.0 //28.2
        @JvmField var TOP = 27.75 //28.2
        @JvmField var BOTTOM = 0.10
        @JvmField var ARM = 3.5
        @JvmField var ARM_TARGET = 3.75
        @JvmField var SPECIMEN_PLACE = 13.5
        @JvmField var CLIMB_ZERO = 9.9
        @JvmField var CLIMB_ONE = 29.0
        @JvmField var CLIMB_TWO = 8.5
        @JvmField var CLIMB_THREE = 27.5
    }

    @Config
    object ClimbPositions {
        @JvmField var TOP = 0.0
        @JvmField var L2 = -22.0
        @JvmField var L3 = -2.0
    }

    @Config
    object ElevatorConstants {
        @JvmField var TICKS_TO_INCHES = 0.0005784991162
        //@JvmField var POSITION_TOLERANCE = 0.25
        @JvmField var POSITION_TOLERANCE = 0.5
        @JvmField var VELOCITY_TOLERANCE = 5.0
        @JvmField var SMALLER_POSITION_TOLERANCE = 0.1
        @JvmField var SMALLER_VELOCITY_TOLERANCE = 0.2

        @JvmField var CLIMB_TICKS_TO_INCHES = 0.004217791411 //1.0 //0.0007893271368
        @JvmField var CLIMB_POSITION_TOLERANCE = 0.1
        @JvmField var CLIMB_VELOCITY_TOLERANCE = 5.0
    }

    @Config
    object VerticalArmPositions {
        @JvmField var INTAKE = 0.0
        @JvmField var SAMPLE = 0.6
        @JvmField var SPECIMEN = 0.93
        @JvmField var AUTO_START = 0.25
    }

    @Config
    object VerticalArmConstants {
        @JvmField var unitsToSeconds = 0.4
        @JvmField var intakeToSpecimen = 0.3
        @JvmField var specimenToIntake = 0.65

        @JvmField var intakeToSample = 0.3
        @JvmField var sampleToIntake = 0.020
    }

    @Config
    object DepositPositions {
        @JvmField var IN = 0.01
        //@JvmField var MID = 0.08
        @JvmField var MID = 0.40
        @JvmField var OUT = 0.90
        /*
        old
        @JvmField var IN = 0.0
        @JvmField var MID = 0.08
        @JvmField var OUT = 1.0
         */

        @JvmField var DEPOSIT_THRESHOLD = 35.0
        @JvmField var TIGHT_DEPOSIT_THRESHOLD = 10.0
        @JvmField var BLUE_TIGHT_DEPOSIT_THRESHOLD = 11.5
    }

    @Config
    object VerticalWristPositions {
        @JvmField var INTAKE = 0.02 //home
        @JvmField var SAMPLE = 0.45 //straight
        @JvmField var SPECIMEN_PICKUP = 0.65 //wall
        @JvmField var SPECIMEN_PLACE = 0.935 //specimen place
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
     * angle 55 units
     * pwm power 94.1%
     * sensitivity high
     *
     * old deposit
     * angle 50 units
     * pwm power 94.1%
     * sensitivity high
     */

}