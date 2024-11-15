package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants.Measurements.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants.Measurements.WHEEL_BASE
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin


class SwerveDrivetrain: Subsystem {
    private var lf: SwerveModule
    private var rf: SwerveModule
    private var lr: SwerveModule
    private var rr: SwerveModule
    private var kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        DrivebaseConstants.Measurements.LF_POS,
        DrivebaseConstants.Measurements.RF_POS,
        DrivebaseConstants.Measurements.LR_POS,
        DrivebaseConstants.Measurements.RR_POS
    )
    //private var imu: IMU
    private var odo: SparkFunOTOS by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(SparkFunOTOS::class.java, DeviceIDs.OTOS)
    }
    private var pose = Pose2d()

    /* Define the offsets from the center of the robot to each wheel */

    private val k = TRACK_WIDTH /2
    private val j = WHEEL_BASE /2

    private val r = arrayOf(
        arrayOf(k, -j), // RF
        arrayOf(k, j), // LF
        arrayOf(-k, -j), // RR
        arrayOf(-k, j), // LR
    )


    constructor(hardwareMap: HardwareMap) {
        val id = DeviceIDs
        lf = SwerveModule(hardwareMap, id.LF_DRIVE_MOTOR, id.LF_TURN_MOTOR, id.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET)
        rf = SwerveModule(hardwareMap, id.RF_DRIVE_MOTOR, id.RF_TURN_MOTOR, id.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        lr = SwerveModule(hardwareMap, id.LR_DRIVE_MOTOR, id.LR_TURN_MOTOR, id.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET)
        rr = SwerveModule(hardwareMap, id.RR_DRIVE_MOTOR, id.RR_TURN_MOTOR, id.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)
        //imu = hardwareMap.get(IMU::class.java, "imu")
        //odo = hardwareMap.get(SparkFunOTOS::class.java, id.OTOS)
        //configureOtos()
    }

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    override fun preUserInitHook(opMode: Wrapper) {
        val id = DeviceIDs
        val hardwareMap = opMode.opMode.hardwareMap
        lf = SwerveModule(hardwareMap, id.LF_DRIVE_MOTOR, id.LF_TURN_MOTOR, id.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET)
        rf = SwerveModule(hardwareMap, id.RF_DRIVE_MOTOR, id.RF_TURN_MOTOR, id.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        lr = SwerveModule(hardwareMap, id.LR_DRIVE_MOTOR, id.LR_TURN_MOTOR, id.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET)
        rr = SwerveModule(hardwareMap, id.RR_DRIVE_MOTOR, id.RR_TURN_MOTOR, id.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)

        configureOtos()
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        val tempPose = odo.position
        pose = Pose2d(tempPose.x, tempPose.y, Rotation2d.fromDegrees(tempPose.h))

        lf.periodic()
        rf.periodic()
        lr.periodic()
        rr.periodic()

    }

    fun FieldCentricDrive(): Lambda {
        return Lambda("field-centric-drive").setInterruptible(true).setExecute(
            firstOrderFieldCentricDrive(
                ChassisSpeeds(
                    -forwardSpeed.asDouble.pow(1) * DrivebaseConstants.Measurements.MAX_VELOCITY,
                    strafeSpeed.asDouble.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
                    turnSpeed.asDouble.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
                )
            )
        )
    }

    /*
    fun periodic() {
        val tempPose = odo.position
        pose = Pose2d(tempPose.x, tempPose.y, Rotation2d.fromDegrees(tempPose.h))

        lf.periodic()
        rf.periodic()
        lr.periodic()
        rr.periodic()

    }
     */

    /**
     * @return radians
     */
    fun getHeading(): Double {
        return getPose().rotation.radians
        //return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }

    fun resetHeading() {
        val currentPose = getPose()
        odo.position = SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, 0.0)
    }

    fun getPose(): Pose2d {
        //val pose = odo.position
        //return Pose2d(pose.x, pose.y, Rotation2d.fromDegrees(pose.h))
        return pose
    }

    /**
     * in/s, deg/s
     */
    fun getVelocity(): ChassisSpeeds {
        val velocity = odo.velocity
        return ChassisSpeeds(velocity.x, velocity.y, velocity.h)
    }

    /**
     * in/s/s, deg/s/s
     */
    fun getAcceleration(): ChassisSpeeds {
        val accel = odo.acceleration
        return ChassisSpeeds(accel.x, accel.y, accel.h)
    }

    fun drive(speeds: ChassisSpeeds) {
        //if (hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) >= 0.01 || speeds.omegaRadiansPerSecond >= 0.01) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds))
        //}
    }

    fun firstOrderDrive(speeds: ChassisSpeeds) {
        val moduleStates = firstOrderInverse(speeds)
        setModuleStates(moduleStates)
    }

    fun firstOrderInverse(speeds: ChassisSpeeds): Array<SwerveModuleState> {
        /* Define the robot velocities */
        val vx = speeds.vxMetersPerSecond
        val vy = speeds.vyMetersPerSecond
        val vw = speeds.omegaRadiansPerSecond

        /* Define array to store module states */
        var moduleStates: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }

        for (i in moduleStates.indices) {
            val vxModule = vx + vw * r[i][1]
            val vyModule = vy + vw * r[i][0]
            val moduleVelocity = hypot(vxModule, vyModule)
            val moduleHeading = atan2(vyModule, vxModule)

            moduleStates[i] = SwerveModuleState(moduleVelocity, Rotation2d(moduleHeading))

        }

        return moduleStates
    }

    fun secondOrderInverse(speeds: ChassisSpeeds, accels: ChassisSpeeds): Array<SwerveModule.SwerveModuleStateAccel> {
        var moduleStates: Array<SwerveModule.SwerveModuleStateAccel> = Array(4) { SwerveModule.SwerveModuleStateAccel() }
        for (i in moduleStates.indices) {
            val m = SimpleMatrix(
                arrayOf(
                    doubleArrayOf(1.0, 0.0, -r[i][0], -r[i][1]),
                    doubleArrayOf(0.0, 1.0, -r[i][1], r[i][0])
                )
            )
            val input = SimpleMatrix(
                arrayOf(
                    doubleArrayOf(
                        accels.vxMetersPerSecond, accels.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond.pow(2.0), accels.omegaRadiansPerSecond
                    )
                )
            )
            val moduleAccels = m.mult(input)

            val vxModule = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond * r[0][1]
            val vyModule = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r[0][0]
            val moduleVelocity = hypot(vxModule, vyModule)
            val moduleHeading = atan2(vyModule, vxModule)

            val m2 = SimpleMatrix(
                arrayOf(
                    doubleArrayOf(cos(moduleHeading), sin(moduleHeading)),
                    doubleArrayOf(-sin(moduleHeading), cos(moduleHeading))
                )
            )
            val final = m2.mult(moduleAccels)

            moduleStates[i] = SwerveModule.SwerveModuleStateAccel(moduleVelocity, moduleHeading, final[0], final[1])
        }

        return moduleStates
    }

    fun secondOrderDrive(speeds: ChassisSpeeds, accels: ChassisSpeeds) {
        val moduleStates = secondOrderInverse(speeds, accels)
        setModuleStatesAccel(moduleStates)
    }


    fun fieldCentricDrive(speeds: ChassisSpeeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())))
    }

    fun firstOrderFieldCentricDrive(speeds: ChassisSpeeds) {
        firstOrderDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())))
    }

    fun setModuleStates(moduleStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        lf.setDesiredState(moduleStates[0])
        rf.setDesiredState(moduleStates[1])
        lr.setDesiredState(moduleStates[2])
        rr.setDesiredState(moduleStates[3])
    }

    fun setModuleStatesAccel(moduleStates: Array<SwerveModule.SwerveModuleStateAccel>) {
        // SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        lf.setDesiredStateAccel(moduleStates[0])
        rf.setDesiredStateAccel(moduleStates[1])
        lr.setDesiredStateAccel(moduleStates[2])
        rr.setDesiredStateAccel(moduleStates[3])
    }

    /**
     * sets the headings of the modules to what they should be at the given chassis speed without
     * drive the motors. Field centric
     */
    fun setModuleHeadings(speeds: ChassisSpeeds) {
        val fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())
        )

        val moduleStates = firstOrderInverse(fieldSpeeds)
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DrivebaseConstants.Measurements.MAX_VELOCITY)
        lf.setDesiredState(moduleStates[0], false)
        rf.setDesiredState(moduleStates[1], false)
        lr.setDesiredState(moduleStates[2], false)
        rr.setDesiredState(moduleStates[3], false)
    }

    /**
     * if all modules are at the desired heading
     * @param tolerance degrees
     */
    fun areModulesAligned(tolerance: Double): Boolean {
        return abs(lf.getDelta()) < tolerance && abs(rf.getDelta()) < tolerance && abs(lr.getDelta()) < tolerance && abs(rr.getDelta()) < tolerance
    }

    fun areModulesAligned(): Boolean {
        return areModulesAligned(DrivebaseConstants.ModuleCoefficients.alignTolerance)
    }

    fun getModuleHeadings(): Array<Double> {
        return arrayOf(lf.getHeading(), rf.getHeading(), lr.getHeading(), rr.getHeading())
    }

    fun getModuleEncoderVoltages(): Array<Double> {
        return arrayOf(lf.getEncoderVoltage(), rf.getEncoderVoltage(), lr.getEncoderVoltage(), rr.getEncoderVoltage())
    }

    fun getDesiredModuleStates(): Array<SwerveModuleState> {
        return arrayOf(lf.getDesiredState(), rf.getDesiredState(), lr.getDesiredState(), rr.getDesiredState())

    }

    fun getDelta(): Array<Double> {
        return arrayOf(lf.getDelta(), rf.getDelta(), lr.getDelta(), rr.getDelta())
    }

    fun test(drive: Double, steer: Double) {
        lf.spin(drive, steer)
        rf.spin(drive, steer)
        lr.spin(drive, steer)
        rr.spin(drive, steer)
    }

    fun stop() {
        lf.stop()
        rf.stop()
        lr.stop()
        rr.stop()
    }

    fun testModule(index: Int, drivePower: Double, steerPower: Double) {
        when(index) {
            0 -> lf.spin(drivePower, steerPower)
            1 -> rf.spin(drivePower, steerPower)
            2 -> lr.spin(drivePower, steerPower)
            3 -> rr.spin(drivePower, steerPower)
            else -> {
                lf.spin(0.0, 0.0)
                rf.spin(0.0, 0.0)
                lr.spin(0.0, 0.0)
                rr.spin(0.0, 0.0)
            }
        }
    }

    private fun configureOtos() {
        val config = DrivebaseConstants.Otos
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // otos.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        odo.setLinearUnit(config.linearUnit)
        // otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        odo.setAngularUnit(config.angularUnit)

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
        // val offset = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
        odo.setOffset(config.offset)

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
        odo.setLinearScalar(config.linearScalar)
        odo.setAngularScalar(config.angularScalar)

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
        // val currentPosition = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
        odo.setPosition(config.startPose)

        // Get the hardware and firmware version
        val hwVersion = SparkFunOTOS.Version()
        val fwVersion = SparkFunOTOS.Version()
        odo.getVersionInfo(hwVersion, fwVersion)
    }


}