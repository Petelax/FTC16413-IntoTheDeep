package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
object DrivebaseConstants {
    @Config
    object ModuleCoefficients {
        @JvmField var KS = 1.5
        @JvmField var KV = 6.82 //7.0
        @JvmField var KA = 0.48 //0.5

        @JvmField var KP = 0.45
        @JvmField var KI = 0.0
        @JvmField var KD = 0.0

        @JvmField var alignTolerance = 15.0
    }

    @Config
    object Measurements {
        const val PI = 3.1415926535897932384626433832795028841971693993751058209
        const val INCHES_TO_METERS = 0.0254
        const val TRACK_WIDTH = 8.50 * INCHES_TO_METERS
        const val WHEEL_BASE = 8.50 * INCHES_TO_METERS
        const val WHEEL_RADIUS = 0.072
        const val TICKS_PER_REV = 8192
        //const val TICKS_TO_INCHES = (PI*2.0*WHEEL_RADIUS) / TICKS_PER_REV
        const val MAX_VELOCITY = 1.8 /* meters per second */
        const val MAX_ACCELERATION = 3.6
        //const val MAX_ACCEL = 15.0 /* meters per second */
        const val MAX_ANGULAR_VELOCITY = 10 //13.5 /* rad per second */
        //const val CENTER_WHEEL_OFFSET = 0.0
        private const val k = TRACK_WIDTH/2
        private const val j = WHEEL_BASE/2

        @JvmField var cachingTolerance = 0.01

        // note: drivebase efficiency prototype 84%

        //const val MAX_ANGULAR_VELOCITY_RAD = 10

        val LF_POS = Translation2d(k, j)
        val RF_POS = Translation2d(k, -j)
        val LR_POS = Translation2d(-k, j)
        val RR_POS = Translation2d(-k, -j)

        /*
        const val LF_OFFSET = 1.2775
        const val RF_OFFSET = 3.4595//0.3179
        const val LR_OFFSET = 6.1118
        const val RR_OFFSET = 4.5067

         */
        const val LF_OFFSET = -5.3141
        const val RF_OFFSET = -2.4581
        const val LR_OFFSET = -5.9195
        const val RR_OFFSET = -4.8038


    }

    @Config
    object Otos {
        @JvmField var offset = SparkFunOTOS.Pose2D(0.0, 0.0, -180.0)
        @JvmField var angularUnit = AngleUnit.DEGREES
        @JvmField var linearUnit = DistanceUnit.INCH

        @JvmField var linearScalar = 1.004 // 1.1188 // 1.011 old, 1.024 new
        @JvmField var angularScalar = 0.99053 //0.9857

        //@JvmField var startPose = SparkFunOTOS.Pose2D(78.0, 7.0, 90.0)
        @JvmField var startPose = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
    }


    /*
    @Config
    object PurePursuit : DrivetrainPIDCoefficients(
       /*
        TranslationKP = 0.3,
        TranslationKI = 0.0,
        TranslationKD = 0.025,

        RotationKP = 15.0,
        RotationKI = 0.0,
        RotationKD = 0.35,

        TranslationPositionTolerance = 0.25,
        TranslationVelocityTolerance = 10.0,

        RotationPositionTolerance = 0.025,
        RotationVelocityTolerance = 1.0,

        MaxAcceleration = 30.0,
        MaxVelocity = 0.05,

        MaxAngularAcceleration = 120.0,
        MaxAngularVelocity = 60.0,

        KF = 0.05

        */
    ) {

        @JvmField var KF = 0.05

        @JvmField var TranslationKP = 0.3
        @JvmField var TranslationKI = 0.0
        @JvmField var TranslationKD = 0.025

        @JvmField var RotationKP = 15.0
        @JvmField var RotationKI = 0.0
        @JvmField var RotationKD = 0.35

        @JvmField var TranslationPositionTolerance = 0.25
        @JvmField var TranslationVelocityTolerance = 10.0

        @JvmField var RotationPositionTolerance = 0.025
        @JvmField var RotationVelocityTolerance = 1.0

        @JvmField var MaxAcceleration = 30.0
        @JvmField var MaxVelocity = 0.05

        @JvmField var MaxAngularAcceleration = 120.0
        @JvmField var MaxAngularVelocity = 60.0

        @JvmField var K_SMOOTH = 0.80
        @JvmField var K_CURVATURE = 0.05
        @JvmField var SPACING = 3.0
        @JvmField var K_FOLLOW_DISTANCE = 10.0
        @JvmField var K_MIN_FOLLOW_DISTANCE = 3.0
    }
     */

    @Config
    object PurePursuit {
        @JvmField
        var K_SMOOTH = 0.80
        @JvmField
        var K_CURVATURE = 3.0
        @JvmField
        var SPACING = 3.0
        @JvmField
        var K_FOLLOW_DISTANCE = 15.0
        @JvmField
        var K_MIN_FOLLOW_DISTANCE = 3.0
    }

    @JvmField var PurePursuitPIDCoefficients = DrivetrainPIDCoefficients(
        KF = 0.0,

        TranslationKP = 0.3,
        TranslationKI = 0.0,
        TranslationKD = 0.03,

        RotationKP = 15.0,
        RotationKI = 0.0,
        RotationKD = 0.35,

        TranslationPositionTolerance = 0.25,
        TranslationVelocityTolerance = 10.0,

        RotationPositionTolerance = 0.025,
        RotationVelocityTolerance = 1.0,

        MaxAcceleration = 30.0,
        MaxVelocity = 0.05,

        MaxAngularAcceleration = 120.0,
        MaxAngularVelocity = 60.0,

    )

    @JvmField var PIDToPosition = DrivetrainPIDCoefficients(
        KF = 0.05,

        TranslationKP = 0.3,
        TranslationKI = 0.0,
        TranslationKD = 0.025,

        RotationKP = 15.0,
        RotationKI = 0.0,
        RotationKD = 0.35,

        TranslationPositionTolerance = 0.25,
        TranslationVelocityTolerance = 10.0,

        RotationPositionTolerance = 0.025,
        RotationVelocityTolerance = 1.0,

        MaxAcceleration = 30.0,
        MaxVelocity = 0.05,

        MaxAngularAcceleration = 120.0,
        MaxAngularVelocity = 60.0,

    )

    /*
    @Config
    object PIDToPosition {
        @JvmField var KF = 0.05

        @JvmField var TranslationKP = 0.3
        @JvmField var TranslationKI = 0.0
        @JvmField var TranslationKD = 0.025

        @JvmField var RotationKP = 15.0
        @JvmField var RotationKI = 0.0
        @JvmField var RotationKD = 0.35

        @JvmField var TranslationPositionTolerance = 0.25
        @JvmField var TranslationVelocityTolerance = 10.0

        @JvmField var RotationPositionTolerance = 0.025
        @JvmField var RotationVelocityTolerance = 1.0

        @JvmField var MaxAcceleration = 30.0
        @JvmField var MaxVelocity = 0.05

        @JvmField var MaxAngularAcceleration = 120.0
        @JvmField var MaxAngularVelocity = 60.0
    }

     */

    @Config
    object DriveHeadingPID {
        @JvmField var KP = 12.0 //15.0
        @JvmField var KI = 0.0
        @JvmField var KD = 0.35

        @JvmField var PositionTolerance = 0.025
        @JvmField var VelocityTolerance = 1.0
    }


}

data class DrivetrainPIDCoefficients(
    @JvmField var KF: Double = 0.05,

    @JvmField var TranslationKP: Double = 0.3,
    @JvmField var TranslationKI: Double = 0.0,
    @JvmField var TranslationKD: Double = 0.025,

    @JvmField var RotationKP: Double = 15.0,
    @JvmField var RotationKI: Double = 0.0,
    @JvmField var RotationKD: Double = 0.35,

    @JvmField var TranslationPositionTolerance: Double = 0.25,
    @JvmField var TranslationVelocityTolerance: Double = 10.0,

    @JvmField var RotationPositionTolerance: Double = 0.025,
    @JvmField var RotationVelocityTolerance: Double = 1.0,

    @JvmField var MaxAcceleration: Double = 30.0,
    @JvmField var MaxVelocity: Double = 0.05,

    @JvmField var MaxAngularAcceleration: Double = 120.0,
    @JvmField var MaxAngularVelocity: Double = 60.0,
)