package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

@Photon
@TeleOp
class OdoTest: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var drive: SwerveDrivetrain
    private lateinit var gamepad: GamepadEx
    //private lateinit var odo: SparkFunOTOS

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        drive = SwerveDrivetrain(hardwareMap)

        //odo = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        //configureOtos()

        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        drive.fieldCentricDrive(ChassisSpeeds(
            gamepad.leftY*DrivebaseConstants.Measurements.MAX_VELOCITY,
            -gamepad.leftX*DrivebaseConstants.Measurements.MAX_VELOCITY,
            -gamepad.rightX*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
        ))

        //drive.test(gamepad.leftY, gamepad.rightX)
        val headings = drive.getModuleHeadings()
        val states = drive.getDesiredModuleStates()
        //val gyro = drive.getHeading()

        //val pos: Pose2D = odo.getPosition()
        //telemetry.addData("thing", drive.getAThing()[0])
        //telemetry.addData("heading", pos.h)
        //telemetry.addData("x", pos.x)
        //telemetry.addData("y", pos.y)
        telemetry.addData("lf heading", headings[0])
        telemetry.addData("lf desired heading", states[0].angle.radians)
        telemetry.addData("rf heading", headings[1])
        telemetry.addData("rf desired heading", states[1].angle.radians)
        telemetry.addData("lr heading", headings[2])
        telemetry.addData("lr desired heading", states[2].angle.radians)
        telemetry.addData("rr heading", headings[3])
        telemetry.addData("rr desired heading", states[3].angle.radians)

        telemetry.addData("ms", elapsedtime.milliseconds())
        elapsedtime.reset()
    }
    /*
    private fun configureOtos() {
        telemetry.addLine("Configuring OTOS...")
        telemetry.update()

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // odo.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        odo.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES)
        // odo.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        odo.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES)

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        val offset = Pose2D(0.0, 0.0, 0.0)
        odo.setOffset(offset)

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        odo.setLinearScalar(1.0)
        odo.setAngularScalar(1.0)

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        odo.calibrateImu()

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        odo.resetTracking()

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        val currentPosition = Pose2D(0.0, 0.0, 0.0)
        odo.setPosition(currentPosition)

        // Get the hardware and firmware version
        val hwVersion = SparkFunOTOS.Version()
        val fwVersion = SparkFunOTOS.Version()
        odo.getVersionInfo(hwVersion, fwVersion)
        telemetry.addLine("OTOS configured! Press start to get position data!")
        telemetry.addLine()
        telemetry.addLine(
            String.format(
                "OTOS Hardware Version: v%d.%d",
                hwVersion.major,
                hwVersion.minor
            )
        )
        telemetry.addLine(
            String.format(
                "OTOS Firmware Version: v%d.%d",
                fwVersion.major,
                fwVersion.minor
            )
        )
        telemetry.update()
    }

     */
}
