package org.firstinspires.ftc.teamcode.subsystems.swerve

import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.cell.RefCell
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants.Measurements.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants.Measurements.WHEEL_BASE
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.utils.PIDController
import org.firstinspires.ftc.teamcode.utils.Telemetry
import java.lang.annotation.Inherited
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.IEEErem
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin

object SwerveDrivetrain : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(Attach::class.java)

    private lateinit var lf: SwerveModule
    private lateinit var rf: SwerveModule
    private lateinit var lr: SwerveModule
    private lateinit var rr: SwerveModule

    private var kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        DrivebaseConstants.Measurements.LF_POS,
        DrivebaseConstants.Measurements.RF_POS,
        DrivebaseConstants.Measurements.LR_POS,
        DrivebaseConstants.Measurements.RR_POS
    )
    private const val k = TRACK_WIDTH /2
    private const val j = WHEEL_BASE /2

    private val r = arrayOf(
        arrayOf(k, -j), // RF
        arrayOf(k, j), // LF
        arrayOf(-k, -j), // RR
        arrayOf(-k, j), // LR
    )

    private var odo by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(SparkFunOTOS::class.java, DeviceIDs.OTOS)
    }

    private var pose = Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0))
    private var headingOffset = Rotation2d()

    val c = DrivebaseConstants.PIDToPosition
    val xController: PIDFController = PIDFController(c.TranslationKP, c.TranslationKI, c.TranslationKD, 0.0)
    val yController: PIDFController = PIDFController(c.TranslationKP, c.TranslationKI, c.TranslationKD, 0.0)
    val headingController: PIDController = PIDController(c.RotationKP, c.RotationKI, c.RotationKD)

    val profiledXController: ProfiledPIDController = ProfiledPIDController(c.TranslationKP, c.TranslationKI, c.TranslationKD, TrapezoidProfile.Constraints(c.MaxVelocity, c.MaxAcceleration))
    val profiledYController: ProfiledPIDController = ProfiledPIDController(c.TranslationKP, c.TranslationKI, c.TranslationKD, TrapezoidProfile.Constraints(c.MaxVelocity, c.MaxAcceleration))
    val profiledHeadingController: PIDController = PIDController(c.RotationKP, c.RotationKI, c.RotationKD)

    val driveHeadingController: PIDController = PIDController(DrivebaseConstants.DriveHeadingPID.KP, DrivebaseConstants.DriveHeadingPID.KI, DrivebaseConstants.DriveHeadingPID.KD)

    override fun preUserInitHook(opMode: Wrapper) {
        val id = DeviceIDs
        val hardwareMap = opMode.opMode.hardwareMap
        lf = SwerveModule(hardwareMap, id.LF_DRIVE_MOTOR, id.LF_TURN_MOTOR, id.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET)
        rf = SwerveModule(hardwareMap, id.RF_DRIVE_MOTOR, id.RF_TURN_MOTOR, id.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        lr = SwerveModule(hardwareMap, id.LR_DRIVE_MOTOR, id.LR_TURN_MOTOR, id.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET)
        rr = SwerveModule(hardwareMap, id.RR_DRIVE_MOTOR, id.RR_TURN_MOTOR, id.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)

        val config = DrivebaseConstants.Otos
        odo.setLinearUnit(config.linearUnit)
        odo.setAngularUnit(config.angularUnit)

        odo.setOffset(config.offset)

        odo.setLinearScalar(config.linearScalar)
        odo.setAngularScalar(config.angularScalar)

        odo.calibrateImu()

        odo.resetTracking()

        odo.position = SparkFunOTOS.Pose2D(pose.x, pose.y, pose.rotation.degrees)
        headingOffset = Rotation2d()

        //headingOffset = Rotation2d()
        /*
        if (opMode.meta.flavor == OpModeMeta.Flavor.AUTONOMOUS) {
            configureOtos()
        }
         */

        headingController.enableContinuousInput(-PI, PI)

        xController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        yController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        headingController.setTolerance(c.RotationPositionTolerance, c.RotationVelocityTolerance)

        profiledHeadingController.enableContinuousInput(-PI, PI)

        profiledXController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        profiledYController.setTolerance(c.TranslationPositionTolerance, c.TranslationVelocityTolerance)
        profiledHeadingController.setTolerance(c.RotationPositionTolerance, c.RotationVelocityTolerance)

        driveHeadingController.enableContinuousInput(-PI, PI)
        driveHeadingController.setTolerance(DrivebaseConstants.DriveHeadingPID.PositionTolerance, DrivebaseConstants.DriveHeadingPID.VelocityTolerance)

        defaultCommand = fieldCentricDrive(
            { opMode.opMode.gamepad1.left_stick_y.toDouble() },
            { opMode.opMode.gamepad1.left_stick_x.toDouble() },
            { opMode.opMode.gamepad1.right_stick_x.toDouble() },
            { opMode.opMode.gamepad1.x },
            { opMode.opMode.gamepad1.a },
            { opMode.opMode.gamepad1.y } )
    }

    override fun preUserInitLoopHook(opMode: Wrapper) {
        //periodic()
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        periodic()
    }

    fun periodic() {
        val tempPose = odo.position
        pose = Pose2d(tempPose.x, tempPose.y, Rotation2d.fromDegrees(tempPose.h).minus(headingOffset))

        lf.periodic()
        rf.periodic()
        lr.periodic()
        rr.periodic()

        /*
        val packet = TelemetryPacket()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading (deg)", pose.rotation.degrees)
        packet.fieldOverlay().setStroke("#3F51B5")
        val rrPose = Pose2d(pose.x-72.0, pose.y-72.0, pose.rotation)
        Drawing.drawRobot(packet.fieldOverlay(), rrPose)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
         */
        Telemetry.robotPose = pose

    }


    fun p2p(setPoint: Pose2d): Lambda {
        return p2p(setPoint, true)
    }

    fun p2p(setPoint: Pose2d, stop: Boolean): Lambda {
        return Lambda("p2p").addRequirements(SwerveDrivetrain)
            .setInit{
                xController.reset()
                yController.reset()
                headingController.reset()
                xController.setPoint = setPoint.x
                yController.setPoint = setPoint.y
                headingController.setpoint = setPoint.rotation.radians
            }
            .setExecute{
                val currentPose = getPose()
                var xFeedback = -xController.calculate(currentPose.x, setPoint.x)
                xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var yFeedback = -yController.calculate(currentPose.y, setPoint.y)
                yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var headingFeedback = -headingController.calculate(currentPose.rotation.radians, setPoint.rotation.radians)
                headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF

                firstOrderFieldCentricDrive(ChassisSpeeds(xFeedback, yFeedback, headingFeedback))
            }
            .setFinish{xController.atSetPoint() && yController.atSetPoint() && headingController.atSetpoint()}
            .setEnd{_ -> stop()}

    }

    /**
     * clamp
     */
    fun cp2p(setPoint: Pose2d): Lambda {
        return Lambda("cp2p").addRequirements(SwerveDrivetrain)
            .setInit{
                xController.reset()
                yController.reset()
                headingController.reset()
                xController.setPoint = setPoint.x
                yController.setPoint = setPoint.y
                headingController.setpoint = setPoint.rotation.radians
            }
            .setExecute{
                val currentPose = getPose()
                var xFeedback = -xController.calculate(currentPose.x, setPoint.x)
                xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var yFeedback = -yController.calculate(currentPose.y, setPoint.y)
                yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var headingFeedback = -headingController.calculate(currentPose.rotation.radians, setPoint.rotation.radians)
                headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF

                val n = max(xFeedback, yFeedback)

                if (n > DrivebaseConstants.PIDToPosition.MaxVelocity) {
                    xFeedback /= n
                    yFeedback /= n
                    //headingFeedback /= n

                    xFeedback *= DrivebaseConstants.PIDToPosition.MaxVelocity
                    yFeedback *= DrivebaseConstants.PIDToPosition.MaxVelocity
                    //headingFeedback *= DrivebaseConstants.PIDToPosition.MaxVelocity
                }

                firstOrderFieldCentricDrive(ChassisSpeeds(xFeedback, yFeedback, headingFeedback))
            }
            .setFinish{xController.atSetPoint() && yController.atSetPoint() && headingController.atSetpoint()}
            .setEnd{_ -> FtcDashboard.getInstance().telemetry.addLine("help"); FtcDashboard.getInstance().telemetry.update(); stop()}

    }


    fun forwardTime(speed: Double, time: Double): Sequential {
        return Sequential(
            Race(
                Wait(time),
                forward(DrivebaseConstants.Measurements.MAX_VELOCITY*speed),
            ),
            stopCmd()
        )
    }

    fun forwardSensor(speed: Double): Lambda {
        return Lambda("drivetrain-sensor-forward").addRequirements(SwerveDrivetrain)
            .setExecute{ drive(ChassisSpeeds(-speed, 0.0, 0.0)) }
            .setFinish{ Deposit.reallyHoldingPiece() }
            .setEnd{ stop() }
    }


    fun pp2p(setPoint: Pose2d): Lambda {
        return Lambda("pp2p").addRequirements(SwerveDrivetrain)
            .setInit{
                profiledXController.reset()
                profiledYController.reset()
                profiledHeadingController.reset()
                profiledXController.setGoal(setPoint.x)
                profiledYController.setGoal(setPoint.y)
                profiledHeadingController.setpoint = setPoint.rotation.radians

                val currentPose = getPose()
                var xFeedback = -profiledXController.calculate(currentPose.x, setPoint.x)
                xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var yFeedback = -profiledYController.calculate(currentPose.y, setPoint.y)
                yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var headingFeedback = -profiledHeadingController.calculate(currentPose.rotation.radians, setPoint.rotation.radians)
                headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF
            }
            .setExecute{
                val currentPose = getPose()
                var xFeedback = -profiledXController.calculate(currentPose.x, setPoint.x)
                xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var yFeedback = -profiledYController.calculate(currentPose.y, setPoint.y)
                yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var headingFeedback = -profiledHeadingController.calculate(currentPose.rotation.radians, setPoint.rotation.radians)
                headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF

                firstOrderFieldCentricDrive(ChassisSpeeds(xFeedback, yFeedback, headingFeedback))
            }
            .setFinish{profiledXController.atGoal() && profiledYController.atGoal() && profiledHeadingController.atSetpoint()}
            .setEnd{_ -> stop()}

    }

    fun bpp2p(setPoint: Pose2d, timeout: Double) : Race {
        return Race(
            null,
            Parallel(
                pp2p(setPoint),
                Wait(0.1),
            ),
            Wait(timeout)
        )
    }

    fun bp2p(setPoint: Pose2d, timeout: Double) : Race {
        return Race(
            null,
            Parallel(
                p2p(setPoint),
                Wait(0.1),
            ),
            Wait(timeout)
        )
    }

    fun bbp2p(setPoint: Pose2d, timeout: Double, stop: Boolean) : Race {
        return Race(
            null,
            Parallel(
                p2p(setPoint, stop),
                Wait(0.1),
            ),
            Wait(timeout)
        )
    }

    fun bcp2p(setPoint: Pose2d, timeout: Double) : Race {
        return Race(
            null,
            Parallel(
                cp2p(setPoint),
                Wait(0.1),
            ),
            Wait(timeout)
        )
    }

    /**
     * sets drivetrain setpoint by accident
     */
    fun alignModules(setPoint: Pose2d): StatefulLambda<RefCell<ChassisSpeeds>> {
        return StatefulLambda("align-modules", RefCell(ChassisSpeeds())).addRequirements(SwerveDrivetrain)
            .setInit{
                state ->
                val currentPose = getPose()

                xController.reset()
                yController.reset()
                headingController.reset()

                xController.setPoint = setPoint.x
                yController.setPoint = setPoint.y
                headingController.setpoint = setPoint.rotation.radians

                var xFeedback = -xController.calculate(currentPose.x, setPoint.x)
                xFeedback += xFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var yFeedback = -yController.calculate(currentPose.y, setPoint.y)
                yFeedback += yFeedback.sign * DrivebaseConstants.PIDToPosition.KF
                var headingFeedback = -headingController.calculate(currentPose.rotation.radians, setPoint.rotation.radians)
                headingFeedback += headingFeedback.sign * DrivebaseConstants.PIDToPosition.KF

                val speeds = ChassisSpeeds(xFeedback, yFeedback, headingFeedback)

                state.accept(speeds)

                setModuleHeadings(speeds)
            }
            .setExecute{
                state ->
                setModuleHeadings(state.get())
            }
            .setFinish{ state -> setModuleHeadings(state.get()); areModulesAligned() }
            .setEnd{ state -> stopTurnServo() }
            //.setFinish{_ -> false}

    }

    fun ap2p(setPoint: Pose2d): Sequential {
        return Sequential(alignModules(setPoint), p2p(setPoint))
    }

    fun forward(speed: Double): Lambda {
        return Lambda("forward").addRequirements(SwerveDrivetrain)
            .setExecute{ drive(ChassisSpeeds(-speed, 0.0, 0.0)) }
            .setFinish{ false }

    }

    fun stopCmd(): Lambda {
        return Lambda("stop").addRequirements(SwerveDrivetrain)
            .setExecute{ stop() }

    }

    fun setPose(pose: Pose2d) {
        headingOffset = Rotation2d()
        odo.position = SparkFunOTOS.Pose2D(pose.x, pose.y, pose.rotation.degrees)
        periodic()
    }

    /**
     * @return radians
     */
    fun getHeading(): Double {
        return getPose().rotation.radians
        //return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }

    fun resetHeadingCommand(): Lambda {
        return Lambda("reset-heading")
            .setInit{ headingOffset = headingOffset.plus(getPose().rotation) }
    }

    fun reset() {
        setPose(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0)))
        headingOffset = Rotation2d()
        headingController.reset()
    }

    fun resetHeading() {
        headingOffset = headingOffset.plus(getPose().rotation)
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

            moduleStates[i] = SwerveModule.SwerveModuleStateAccel(
                moduleVelocity,
                moduleHeading,
                final[0],
                final[1]
            )
        }

        return moduleStates
    }

    fun secondOrderDrive(speeds: ChassisSpeeds, accels: ChassisSpeeds) {
        val moduleStates = secondOrderInverse(speeds, accels)
        setModuleStatesAccel(moduleStates)
    }


    fun fieldCentricDrive(speeds: ChassisSpeeds) {
        drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())))
    }

    fun firstOrderFieldCentricDrive(speeds: ChassisSpeeds) {
        firstOrderDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            Rotation2d(getHeading())))
    }

    fun normalizeRadians(rads: Double): Double {
        return rads.IEEErem(2 * Math.PI)
    }

    fun fieldCentricDrive(strafeSpeed: DoubleSupplier, forwardSpeed: DoubleSupplier, turnSpeed: DoubleSupplier, basket: BooleanSupplier, observationZone: BooleanSupplier, highRung: BooleanSupplier) : Lambda {
        return Lambda("field-centric-drive").addRequirements(SwerveDrivetrain)
            .setExecute{

                var turnPower = turnSpeed.asDouble.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
                val totalDesiredActions = if(basket.asBoolean) {1} else {0} + if(observationZone.asBoolean) {1} else {0} + if(highRung.asBoolean) {1} else {0}
                if (totalDesiredActions > 1) {
                    ;
                } else if (basket.asBoolean) {
                    turnPower = -driveHeadingController.calculate(normalizeRadians(getHeading()), normalizeRadians(Math.toRadians(-135.0)))
                } else if (observationZone.asBoolean) {
                    turnPower = -driveHeadingController.calculate(normalizeRadians(getHeading()), normalizeRadians(Math.toRadians(-90.0)))
                } else if (highRung.asBoolean) {
                    turnPower = -driveHeadingController.calculate(normalizeRadians(getHeading()), normalizeRadians(Math.toRadians(90.0)))
                }

                firstOrderFieldCentricDrive(
                    ChassisSpeeds(
                        -forwardSpeed.asDouble.pow(3) * DrivebaseConstants.Measurements.MAX_VELOCITY,
                        strafeSpeed.asDouble.pow(3)*DrivebaseConstants.Measurements.MAX_VELOCITY,
                        turnPower
                    )
                )}
            .setFinish{false}

    }

    fun kill(): Lambda {
        return Lambda("kill-drivetrain").addRequirements(SwerveDrivetrain)
            .setInit{lf.kill(); lr.kill(); rf.kill(); rr.kill()}
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

    fun stopTurnServo() {
        lf.stopTurnServo()
        rf.stopTurnServo()
        lr.stopTurnServo()
        rr.stopTurnServo()
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

    private fun configureOtos(startPose: SparkFunOTOS.Pose2D = DrivebaseConstants.Otos.startPose) {
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
        odo.setPosition(startPose)

        // Get the hardware and firmware version
        val hwVersion = SparkFunOTOS.Version()
        val fwVersion = SparkFunOTOS.Version()
        odo.getVersionInfo(hwVersion, fwVersion)
    }



}