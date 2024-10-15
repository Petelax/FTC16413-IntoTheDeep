package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrive
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import kotlin.math.pow

@TeleOp
class CommandTeleOp: CommandOpMode() {
    private lateinit var drive: SwerveDrivetrain
    private lateinit var dashboard: FtcDashboard
    private var lastTime = System.nanoTime()
    private lateinit var gamepad: GamepadEx

    override fun initialize() {
        drive = SwerveDrivetrain(hardwareMap)
        dashboard = FtcDashboard.getInstance()
        gamepad = GamepadEx(gamepad1)

        val packet = TelemetryPacket()
        packet.addLine("init")
        dashboard.sendTelemetryPacket(packet)

        drive.defaultCommand = FieldCentricDrive(
                drive,
                { gamepad.leftX },
                { gamepad.leftY },
                { gamepad.rightX },
                { false },
                { true }
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

        //drive.drive(ChassisSpeeds(1.0, 0.0, 0.0))

        lastTime = currentTime
        super.run()
    }
}