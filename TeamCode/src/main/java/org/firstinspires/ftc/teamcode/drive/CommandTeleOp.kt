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
import org.firstinspires.ftc.teamcode.commands.subsystems.HorizontalArmExtend
import org.firstinspires.ftc.teamcode.commands.subsystems.HorizontalArmRetract
import org.firstinspires.ftc.teamcode.commands.subsystems.HorizontalExtensionCommand
import org.firstinspires.ftc.teamcode.commands.subsystems.HorizontalExtensionPIDCommand
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.Drawing

@TeleOp
class CommandTeleOp: CommandOpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drive: SwerveDrivetrain
    private lateinit var elevator: Elevator
    private lateinit var horizontalExtension: HorizontalExtension
    private lateinit var horizontalArm: HorizontalArm
    private lateinit var dashboard: FtcDashboard
    private var lastTime = System.nanoTime()
    private lateinit var driveOp: GamepadEx
    private lateinit var toolOp: GamepadEx

    override fun initialize() {
        drive = SwerveDrivetrain(hardwareMap)
        elevator = Elevator(hardwareMap)
        horizontalExtension = HorizontalExtension(hardwareMap)
        horizontalArm = HorizontalArm(hardwareMap)
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

        horizontalExtension.defaultCommand = HorizontalExtensionCommand(horizontalExtension) { -toolOp.rightY }

    }

    override fun run() {
        val currentTime = System.nanoTime()

        for (hub in hubs) {
            hub.clearBulkCache()
        }

        //val pose = drive.getPose()

        val packet = TelemetryPacket()
        /*
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("elevator height", elevator.getPosition())

        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), pose)

         */

        GamepadButton(toolOp, GamepadKeys.Button.A).whenPressed(HorizontalArmRetract(horizontalExtension, horizontalArm))
        //GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(HorizontalExtensionPIDCommand(horizontalExtension, 6.0))
        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(HorizontalArmExtend(horizontalExtension, horizontalArm))

        GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM))
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT).whenPressed(ElevatorPIDCommand(elevator, 12.0))
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP))


        super.run()

        packet.put("ms", (currentTime-lastTime)/1E6)
        packet.put("ms pose", (currentTime-lastTime)/1E6)
        dashboard.sendTelemetryPacket(packet)

        lastTime = currentTime
    }
}