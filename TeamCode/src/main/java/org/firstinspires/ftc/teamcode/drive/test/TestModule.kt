package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveModule

@Photon
@TeleOp(group = "test")
class TestModule: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    //private lateinit var drive: SwerveDrivetrain
    private lateinit var module: SwerveModule
    private lateinit var rr: SwerveModule
    private lateinit var gamepad: GamepadEx

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        val id = DeviceIDs
        module = SwerveModule(hardwareMap, id.RF_DRIVE_MOTOR, id.RF_TURN_MOTOR, id.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET)
        rr = SwerveModule(hardwareMap, id.RR_DRIVE_MOTOR, id.RR_TURN_MOTOR, id.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET)
        //drive = SwerveDrivetrain(hardwareMap)
        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        module.spin(0.0, gamepad.rightX)
        rr.spin(0.0, gamepad.rightX)

        //drive.setModuleHeadings(Rotation2d(), Rotation2d(), Rotation2d(), Rotation2d())

        /*
        drive.drive(ChassisSpeeds(
            -gamepad.leftY.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            -gamepad.leftX.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            -gamepad.rightX.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
        ))
         */

        telemetry.addData("delta rf", module.getDelta())
        telemetry.addData("rf heading", module.getHeading())
        telemetry.addData("rf voltage", module.getEncoderVoltage())
        telemetry.addData("rf desired heading", module.getDesiredState())

        //drive.test(gamepad.leftY, gamepad.rightX.49s for 30in)
        //telemetry.addData("thing", drive.getDelta()[0])

        /*
        telemetry.addData("heading rad", gyro)
        telemetry.addData("lf heading", headings[0])
        telemetry.addData("lf voltage", voltages[0])
        telemetry.addData("lf desired heading", states[0].angle.radians)

        telemetry.addData("rf heading", headings[1])
        telemetry.addData("rf voltage", voltages[1])
        telemetry.addData("rf desired heading", states[1].angle.radians)

        telemetry.addData("lr heading", headings[2])
        telemetry.addData("lr voltage", voltages[2])
        telemetry.addData("lr desired heading", states[2].angle.radians)

        telemetry.addData("rr heading", headings[3])
        telemetry.addData("rr voltage", voltages[3])
        telemetry.addData("rr desired heading", states[3].angle.radians)

         */

        telemetry.addData("ms", elapsedtime.milliseconds())
        elapsedtime.reset()
    }}
