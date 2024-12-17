package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.swerve.SwerveDrivetrain

@Disabled
@Photon
@Autonomous(group = "test")
class FFAuto: LinearOpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var drive: SwerveDrivetrain
    private lateinit var gamepad: GamepadEx

    override fun runOpMode() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        drive = SwerveDrivetrain(hardwareMap)
        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()

        waitForStart()

        val constraints = TrapezoidProfile.Constraints(DrivebaseConstants.Measurements.MAX_VELOCITY, DrivebaseConstants.Measurements.MAX_ACCELERATION)
        var profile = TrapezoidProfile(constraints, TrapezoidProfile.State(8.0, 1.0))

        elapsedtime.reset()
        var lastTime = elapsedtime.milliseconds()
        while (profile.isFinished(lastTime)) {
            val currentTime = elapsedtime.milliseconds()

            val state = profile.calculate(currentTime/1000.0)

            drive.drive(ChassisSpeeds(
                state.velocity,
                0.0,
                0.0
            ))

            //drive.test(gamepad.leftY, gamepad.rightX)
            val headings = drive.getModuleHeadings()
            val states = drive.getDesiredModuleStates()
            val gyro = drive.getHeading()
            telemetry.addData("thing", drive.getDelta()[0])
            telemetry.addData("heading", gyro)
            telemetry.addData("lf heading", headings[0])
            telemetry.addData("lf desired heading", states[0].angle.radians)
            telemetry.addData("rf heading", headings[1])
            telemetry.addData("rf desired heading", states[1].angle.radians)
            telemetry.addData("lr heading", headings[2])
            telemetry.addData("lr desired heading", states[2].angle.radians)
            telemetry.addData("rr heading", headings[3])
            telemetry.addData("rr desired heading", states[3].angle.radians)

            telemetry.addData("ms", currentTime-lastTime)
            lastTime = currentTime

        }

    }

}