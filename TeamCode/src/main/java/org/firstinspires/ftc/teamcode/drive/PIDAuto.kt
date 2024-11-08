package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.PIDToPosition
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

@Config
@Autonomous
class PIDAuto: CommandOpMode() {
    private lateinit var drive: SwerveDrivetrain
    private lateinit var dashboard: FtcDashboard

    private var lastTime = System.nanoTime()
    //@JvmField var pose = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))
    @JvmField var pose = Pose2d(24.0, 0.0, Rotation2d.fromDegrees(0.0))

    override fun initialize() {
        //telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drive = SwerveDrivetrain(hardwareMap)
        sleep(700)
        dashboard = FtcDashboard.getInstance()


        val packet = TelemetryPacket()
        packet.addLine("init")
        dashboard.sendTelemetryPacket(packet)

        val help = TelemetryPacket()
        help.addLine("help")

        schedule(
            SequentialCommandGroup(
                SequentialCommandGroup(),
                WaitCommand(10),
                //PIDToPosition(drive, pose),
                //PIDToPosition(drive, Pose2d(24.0, 24.0, Rotation2d.fromDegrees(0.0))),

                PIDToPosition(drive, Pose2d(0.0, 48.0, Rotation2d.fromDegrees(0.0))),
                InstantCommand({dashboard.sendTelemetryPacket(help)}),
                WaitCommand(100)
            ),
        )

    }

    override fun run() {
        val currentTime = System.nanoTime()
        val pose = drive.getPose()
        val packet = TelemetryPacket()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("ms", (currentTime-lastTime)/1E9)
        dashboard.sendTelemetryPacket(packet)
        telemetry.update()

        //drive.drive(ChassisSpeeds(1.0, 0.0, 0.0))

        lastTime = currentTime
        super.run()
    }

}