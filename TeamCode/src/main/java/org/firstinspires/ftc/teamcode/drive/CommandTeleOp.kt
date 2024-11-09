package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drivebase.FieldCentricDrive
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

@TeleOp
class CommandTeleOp: CommandOpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drive: SwerveDrivetrain
    private lateinit var elevator: Elevator
    private lateinit var dashboard: FtcDashboard
    private var lastTime = System.nanoTime()
    private lateinit var driveOp: GamepadEx
    private lateinit var toolOp: GamepadEx

    override fun initialize() {
        drive = SwerveDrivetrain(hardwareMap)
        elevator = Elevator(hardwareMap)
        dashboard = FtcDashboard.getInstance()
        driveOp = GamepadEx(gamepad1)
        toolOp = GamepadEx(gamepad2)

        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        val packet = TelemetryPacket()
        packet.addLine("init")
        dashboard.sendTelemetryPacket(packet)

        drive.defaultCommand = FieldCentricDrive(
                drive,
                { driveOp.leftX },
                { driveOp.leftY },
                { driveOp.rightX },
                { false },
                { true }
        )

        elevator.defaultCommand = ElevatorCommand(elevator) { toolOp.leftY }

    }

    override fun run() {
        val currentTime = System.nanoTime()

        for (hub in hubs) {
            hub.clearBulkCache()
        }

        val pose = drive.getPose()
        val packet = TelemetryPacket()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("elevator height", elevator.getPosition())
        packet.put("ms", (currentTime-lastTime)/1E6)
        dashboard.sendTelemetryPacket(packet)

        GamepadButton(toolOp, GamepadKeys.Button.A).whenPressed(ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM))
        GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(ElevatorPIDCommand(elevator, 12.0))
        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP))

        lastTime = currentTime
        super.run()
    }
}