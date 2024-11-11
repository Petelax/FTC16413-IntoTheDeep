package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import kotlin.math.pow

@TeleOp
class TeleOp: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var drive: SwerveDrivetrain
    private lateinit var gamepad: GamepadEx
    private lateinit var elevator: Elevator
    private lateinit var verticalArm: VerticalArm

    //private lateinit var voltage: PhotonLynxVoltageSensor

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        drive = SwerveDrivetrain(hardwareMap)
        elevator = Elevator(hardwareMap)
        verticalArm = VerticalArm(hardwareMap)
        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        elapsedtime.reset()
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        val cacheTime = elapsedtime.milliseconds()
        telemetry.addData("ms cache", cacheTime)

        drive.firstOrderFieldCentricDrive(ChassisSpeeds(
            -gamepad.leftY.pow(1) *DrivebaseConstants.Measurements.MAX_VELOCITY,
            gamepad.leftX.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            gamepad.rightX.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
        ))

        val driveTime = elapsedtime.milliseconds()
        telemetry.addData("ms drive", driveTime-cacheTime)

        elevator.setSpeed(-gamepad2.left_stick_y.toDouble())

        if (gamepad2.a) {
            verticalArm.setPosition(VerticalConstants.DepositPositions.INTAKE)
        }
        if (gamepad2.y) {
            verticalArm.setPosition(VerticalConstants.DepositPositions.SAMPLE)
        }
        if (gamepad2.b) {
            verticalArm.setPosition(VerticalConstants.DepositPositions.SPECIMEN)
        }

        val subsystemTime = elapsedtime.milliseconds()
        telemetry.addData("ms subsystem", subsystemTime-driveTime)

        /*
        drive.drive(ChassisSpeeds(
            -gamepad.leftY.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            -gamepad.leftX.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            -gamepad.rightX.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
        ))
         */

        /*
        if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
            drive.resetHeading()
        }

         */

        //telemetry.addData("voltage", voltage.cachedVoltage)



        val pose = drive.getPose()
        telemetry.addData("x", pose.x)
        telemetry.addData("y", pose.y)
        telemetry.addData("heading deg", pose.rotation.degrees)

        val poseTime = elapsedtime.milliseconds()
        telemetry.addData("ms pose", poseTime-subsystemTime)

        /*
        val vel = drive.getVelocity()
        telemetry.addData("vel x", vel.vxMetersPerSecond)
        telemetry.addData("vel y", vel.vyMetersPerSecond)
        telemetry.addData("vel heading deg", vel.omegaRadiansPerSecond)
        telemetry.addData("vel magnitude", hypot(vel.vyMetersPerSecond, vel.vxMetersPerSecond))

        //drive.test(gamepad.leftY, gamepad.rightX.49s for 30in)
        val headings = drive.getModuleHeadings()
        val voltages = drive.getModuleEncoderVoltages()
        val states = drive.getDesiredModuleStates()
        val gyro = drive.getHeading()
        //telemetry.addData("thing", drive.getDelta()[0])

        telemetry.addData("heading rad", gyro)
        telemetry.addData("lf heading", headings[0])
        //telemetry.addData("lf voltage", voltages[0])
        telemetry.addData("lf desired heading", states[0].angle.radians)

        telemetry.addData("rf heading", headings[1])
        //telemetry.addData("rf voltage", voltages[1])
        telemetry.addData("rf desired heading", states[1].angle.radians)

        telemetry.addData("lr heading", headings[2])
        //telemetry.addData("lr voltage", voltages[2])
        telemetry.addData("lr desired heading", states[2].angle.radians)

        telemetry.addData("rr heading", headings[3])
        //telemetry.addData("rr voltage", voltages[3])
        telemetry.addData("rr desired heading", states[3].angle.radians)

         */

        //CommandScheduler.getInstance().run()
        drive.periodic()

        val schedulerTime = elapsedtime.milliseconds()
        telemetry.addData("ms scheduler", schedulerTime-poseTime)

        elevator.periodic()

        val scheduler1Time = elapsedtime.milliseconds()
        telemetry.addData("ms scheduler 1", scheduler1Time-schedulerTime)

        telemetry.addData("ms", elapsedtime.milliseconds())

    }}